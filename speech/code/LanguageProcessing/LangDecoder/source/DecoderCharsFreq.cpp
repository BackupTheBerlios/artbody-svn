#include <stdio.h>
#include <string>

#include <vector>
#include <algorithm>
#include <functional>

#include "glob.h"
#include "Xi2.h"
#include "DecoderCharsFreq.h"

namespace lang {

    mt::Real CharsFreqDecoder::xi_2_255_095 = 45.0;
    //
    // struct CharsFreqDecoder 
    //    
    bool CharsFreqDecoder::_LangDistr::init(const std::string& file_in)
    {
        unsigned char* file_data = NULL;
        mt::UInt data_len = 0;
        // read file data 
        if (!CharsFreqDecoder::_read(file_in, file_data, data_len)) {
            return false;
        }

        // detect 'space' character - most frequent character
        unsigned char space_char = _detectSpaceChar(file_data, data_len);        

        // collect chars frequencies, skip repeating spaces
        bool bPrevSpace = false;
        for (mt::UInt data_iter = 0; data_iter < data_len; data_iter++) {
            unsigned char cur_char = file_data[data_iter];
            if (cur_char != space_char) {
                charsFreqVector[cur_char]++;
                bPrevSpace = false;
            } else {
                // skip all spaces
                continue;
                if (!bPrevSpace) { // skip only repeating spaces
                    charsFreqVector[cur_char]++;                
                }
                bPrevSpace = true;
            }
        }

        // sort frequencies 
        std::sort(charsFreqVector.begin(), charsFreqVector.end(), std::greater<mt::UInt>());

        
        delete [] file_data;

        return true;
    }

    unsigned char CharsFreqDecoder::_LangDistr::_detectSpaceChar(unsigned char* file_data, mt::UInt data_len)
    {
        unsigned char space_char = 0;
        mt::UInt most_freq = 0;
        mt::UInt char_freq[256];
        memset(char_freq, 0, sizeof (char_freq));
        for (mt::UInt data_iter = 0; data_iter < data_len; data_iter++) {
            unsigned char cur_char = file_data[data_iter];
            char_freq[cur_char]++;
            if (char_freq[cur_char] > most_freq) {
                space_char = cur_char;
                most_freq = char_freq[cur_char];
            }
        }
        return space_char;
    }

    //
    // Class lang::CharsFreqDecoder
    //
    CharsFreqDecoder::CharsFreqDecoder()
    {
        ;
    }

    bool CharsFreqDecoder::trainLanguage(const std::string& file_in, LanguageID language)
    {
        _LangDistr& inp_lang_distr = m_trainedLanguages[language];
        return inp_lang_distr.init(file_in);
    }

    LanguageID CharsFreqDecoder::resolveLanguage(const std::string& file_in)
    {
        _LangDistr inp_lang_distr;
        inp_lang_distr.init(file_in);


        LanguageID best_lang = Language_ENG;
        mt::Real best_x2_value = mt::INFINITY;
        for (int lang_id = 0; lang_id < Language_MAX; lang_id++) {
            _LangDistr& cur_lang_distr = m_trainedLanguages[lang_id];
            
            st::Xi2Calculator x2Calculator(inp_lang_distr.charsFreqVector, cur_lang_distr.charsFreqVector);
            mt::Real x2_value = x2Calculator.getValue();
            if (x2_value < best_x2_value) {
                best_x2_value = x2_value;
                best_lang = static_cast<LanguageID>(lang_id);
            }
        }

        glob::assert(best_x2_value < xi_2_255_095);

        return best_lang;
    }

    //
    // Internal methods
    //   


} // namespace lang
                                                                                 