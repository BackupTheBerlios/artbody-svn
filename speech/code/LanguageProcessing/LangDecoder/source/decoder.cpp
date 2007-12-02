#include <stdio.h>
#include <string>

#include <algorithm>

#include "LangDecoder.h"
#include "GaussDistr.h"
#include "NGram.h"

namespace lang {

    //
    // Class lang::Decoder::_CharFreqIndex
    //
    Decoder::_CharFreqIndex::_CharFreqIndex()
    {
        // init chars' table
        m_charFreq_index.resize(256);
        for (int char_idx = 0; char_idx < 256; char_idx++) {
            _CharDescr newElem(static_cast<unsigned char>(char_idx), 0);
            m_charFreq_index[char_idx] = newElem;
        }
    }

    void Decoder::_CharFreqIndex::putChar(unsigned char c)
    {
        _CharDescr& elem = m_charFreq_index[c];
        elem.pair.second++;
    }

    void Decoder::_CharFreqIndex::sort(void)
    {
        std::sort(m_charFreq_index.begin(), m_charFreq_index.end(), _CharDescr::compareByFreq);

        size_t index_size = m_charFreq_index.size();
        for (unsigned int elem_idx = 0; elem_idx < index_size; elem_idx++) {
            m_charFreq_index[elem_idx].pair.second = elem_idx;
        }

        std::sort(m_charFreq_index.begin(), m_charFreq_index.end(), _CharDescr::compareByChar);
    }

    int Decoder::_CharFreqIndex::getCharIndex(unsigned char c)
    {
        _CharDescr char_to_find(c, 0);

//        _CharDescr* p_char_descr = (_CharDescr*)bsearch((void*)&char_to_find, (void*)&m_charFreq_index, 
//                                    m_charFreq_index.size(), sizeof(_CharDescr), 
//                                    (int (*)(const void*, const void*))_CharDescr::bsCompareByChar);
//
//        if (p_char_descr != NULL) {
//            return p_char_descr->pair.second;
//        }
        std::vector<_CharDescr>::const_iterator find_iter = find(m_charFreq_index.begin(), m_charFreq_index.end(), char_to_find);
        if (find_iter != m_charFreq_index.end()) {
            return find_iter->pair.second;
        }

        return static_cast<int>(m_charFreq_index.size());
    }


    //
    // Class lang::Decoder
    //
    Decoder::Decoder()
    {
        ;
    }

    bool Decoder::trainLanguage(const std::string& file_in, LanguageID language)
    {
        unsigned char* file_data = NULL;
        unsigned int data_len = 0;
        // read file data 
        if (!_read(file_in, file_data, data_len)) {
            return false;
        }        
        
        _LangDescr& lang_descr = m_trainedLanguages[language];        
        
        _fillFreqVector(lang_descr.index, file_data, data_len);
        
        _trainLanguageModel(lang_descr, file_data, data_len);

        delete [] file_data;

        return true;
    }

    LanguageID Decoder::resolveLanguage(const std::string& file_in)
    {
        unsigned char* file_data = NULL;
        unsigned int data_len = 0;
        // read file data 
        if (!_read(file_in, file_data, data_len)) {
            return Language_MAX;
        }
        // calculate frequency distribution
        _CharFreqIndex encodedIndex;
        _fillFreqVector(encodedIndex, file_data, data_len);

        // resolve language
        LanguageID bestLangId;
        mt::Real bestLangProb;
        for (int langId = 0; langId < Language_MAX; langId++) {            
            st::BiGram<int>& testLangModel = m_trainedLanguages[langId].langModel;
            mt::Real langLogProb = _calcInputProbabilityWithLang(testLangModel, encodedIndex, file_data, data_len);
            if (langId == 0) {
                bestLangId = static_cast<LanguageID>(langId);
                bestLangProb = langLogProb;
            } else {
                if (langLogProb > bestLangProb) {
                    bestLangId = static_cast<LanguageID>(langId);
                    bestLangProb = langLogProb;
                }
            }
        }
        return bestLangId;
    }

    //
    // Internal methods
    //
    bool Decoder::_read(const std::string& file_in, unsigned char*& file_data, unsigned int& data_len)
    {
        // open file
        FILE* fp = fopen(file_in.c_str(), "r");
        if (fp == NULL) {
            return false;
        }

        // calc size of file
        fseek(fp, 0, SEEK_END);
        unsigned int fileSize = ftell(fp);
        fseek(fp, 0, SEEK_SET);

        // alloc storage vector
        data_len = fileSize / sizeof(char);
        file_data = new unsigned char[data_len * sizeof(char)];
        if (file_data == NULL) {
            return false;
        }
        
        // read data into storage vector
        unsigned int nChars = 0;
        while (!feof(fp) && (nChars < data_len)){
            unsigned char cur_c = static_cast<unsigned char>(fgetc(fp));    
            file_data[nChars++] = cur_c;
        }

        fclose(fp);

        return true;
    }

    void Decoder::_fillFreqVector(_CharFreqIndex& index, const unsigned char* file_data, unsigned int data_len)
    {
        for (unsigned int elem_idx = 0; elem_idx < data_len; elem_idx++) {
            index.putChar(file_data[elem_idx]);
        }

        index.sort();
    }

    void Decoder::_trainLanguageModel(_LangDescr& lang_descr, const unsigned char* file_data, unsigned int data_len)
    {        
        unsigned char space_sym = ' ';

        std::vector<int> one_train_vect;
        
        // train language model basing on frequency of symbol instead of symbol itself
        for (unsigned int data_elem_idx = 0; data_elem_idx < data_len; data_elem_idx++) {            
            unsigned char data_elem = file_data[data_elem_idx];
            // space is the delimiter of words
            if (data_elem != space_sym) {
                int elem_freq_nmb = lang_descr.index.getCharIndex(data_elem);
                one_train_vect.push_back(elem_freq_nmb);
            } else {
                if (one_train_vect.size() > 0) {
                    lang_descr.langModel.train(one_train_vect);
                    one_train_vect.clear();
                }
            }
        }

        lang_descr.langModel.finishTraining();
    }

    mt::Real Decoder::_calcInputProbabilityWithLang(st::BiGram<int>& testModel, _CharFreqIndex& encodedIndex, const unsigned char* file_data, unsigned int data_len)
    {
        unsigned char space_sym = encodedIndex.getIndexChar(0);

        mt::Real input_prob = 0.f;
        std::vector<int> one_word_vect;
        for (unsigned int data_elem_idx = 0; data_elem_idx < data_len; data_elem_idx++) {            
            unsigned char data_elem = file_data[data_elem_idx];
            // space is the delimiter of words
            if (data_elem != space_sym) {
                int elem_freq_nmb = encodedIndex.getCharIndex(data_elem);
                one_word_vect.push_back(elem_freq_nmb);
            } else {
                if (one_word_vect.size() > 0) {
                    input_prob += testModel.calcSequenceLogProbability(one_word_vect);
                    one_word_vect.clear();
                }
            }
        }

        return input_prob;
    }

} // namespace lang
                                                                                 