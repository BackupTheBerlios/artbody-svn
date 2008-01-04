#include <stdio.h>
#include <string>

#include <vector>

#include "glob.h"
#include "DecoderSpaceFreq.h"

namespace st {
    class Xi2Calculator {
    public:
        Xi2Calculator(std::vector<mt::UInt>& data1, std::vector<mt::UInt>& data2);
        mt::Real getValue(void) { return m_xi2Value; }
    private:
        mt::UInt _getTotalElems(std::vector<mt::UInt>& data);

        mt::Real m_xi2Value;
    };

    Xi2Calculator::Xi2Calculator(std::vector<mt::UInt>& data1, std::vector<mt::UInt>& data2):
    m_xi2Value(0.0)
    {
        if (data1.size() != data2.size()) {
            glob::assert(false);
            return;
        }

        //get total amount of elements in first distribution
        mt::UInt n_elems_1 = _getTotalElems(data1);
        mt::UInt n_elems_2 = _getTotalElems(data2);

        mt::UInt n_groups = static_cast<mt::UInt>(data1.size());
        for (mt::UInt group_idx = 0; group_idx < n_groups; group_idx++) {
            mt::UInt n_group_elems_1 = data1[group_idx];
            mt::UInt n_group_elems_2 = data2[group_idx];

            mt::Real one_div_gr1_gr2 = 1.0 / (n_group_elems_1 + n_group_elems_2);
            mt::Real gr1_div_n1 = static_cast<mt::Real>(n_group_elems_1) / n_elems_1;
            mt::Real gr1_div_n2 = static_cast<mt::Real>(n_group_elems_2) / n_elems_2;
            mt::Real g1_d_n1_g2_d_n2_2 = (gr1_div_n1 - gr1_div_n2) * (gr1_div_n1 - gr1_div_n2);
            m_xi2Value += one_div_gr1_gr2 * g1_d_n1_g2_d_n2_2;
        }
        m_xi2Value *= n_elems_1 * n_elems_2;
    }

    mt::UInt Xi2Calculator::_getTotalElems(std::vector<mt::UInt>& data)
    {
        mt::UInt n_total_elems = 0;
        mt::UInt data_size = static_cast<mt::UInt>(data.size());
        for (mt::UInt data_iter = 0; data_iter < data_size; data_iter++) {
            n_total_elems += data[data_iter];
        }
        return n_total_elems;
    }
} // namespace st

namespace lang {

    mt::Real SpaceFreqDecoder::xi_2_14_095 = 23.685;
    //
    // struct SpaceFreqDecoder 
    //    
    bool SpaceFreqDecoder::_LangDistr::init(const std::string& file_in)
    {
        unsigned char* file_data = NULL;
        mt::UInt data_len = 0;
        // read file data 
        if (!SpaceFreqDecoder::_read(file_in, file_data, data_len)) {
            return false;
        }

        // detect 'space' character - most frequent character
        unsigned char space_char = _detectSpaceChar(file_data, data_len);        

        // detect words delimited by 'space' char
        _detectWords(file_data, data_len, space_char);

        delete [] file_data;

        return true;
    }

    void SpaceFreqDecoder::_LangDistr::copyToVector(std::vector<mt::UInt>& group_vector)
    {
        group_vector.resize(MAX_WORD_LEN);
        for (mt::UInt group_iter = 0; group_iter < MAX_WORD_LEN; group_iter++) {
            group_vector[group_iter] = nWordsInGroup[group_iter];
        }
    }

    unsigned char SpaceFreqDecoder::_LangDistr::_detectSpaceChar(unsigned char* file_data, mt::UInt data_len)
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

    void SpaceFreqDecoder::_LangDistr::_detectWords(unsigned char* file_data, mt::UInt data_len, unsigned char space_char)
    {
        mt::UInt word_len = 0;
        for (mt::UInt data_iter = 0; data_iter < data_len; data_iter++) {
            unsigned char cur_char = file_data[data_iter];
            if (cur_char == space_char) { // probably 'end-of-word' found, check it
                if (word_len > 0) { // end-of-word
                    word_len = (word_len < MAX_WORD_LEN) ? word_len : MAX_WORD_LEN;
                    nWordsInGroup[word_len - 1]++;
                    nTotalWords++;
                } else { // continios spaces
                    continue;                        
                }
                word_len = 0;
            } else {
                word_len++;
            }
        }
    }

    //
    // Class lang::SpaceFreqDecoder
    //
    SpaceFreqDecoder::SpaceFreqDecoder()
    {
        ;
    }

    bool SpaceFreqDecoder::trainLanguage(const std::string& file_in, LanguageID language)
    {
        _LangDistr& inp_lang_distr = m_trainedLanguages[language];
        return inp_lang_distr.init(file_in);
    }

    LanguageID SpaceFreqDecoder::resolveLanguage(const std::string& file_in)
    {
        _LangDistr inp_lang_distr;
        inp_lang_distr.init(file_in);

        std::vector<mt::UInt> inp_data_vect;
        inp_lang_distr.copyToVector(inp_data_vect);

        LanguageID best_lang = Language_ENG;
        mt::Real best_x2_value = mt::INFINITY;
        for (int lang_id = 0; lang_id < Language_MAX; lang_id++) {
            _LangDistr& cur_lang_distr = m_trainedLanguages[lang_id];
            
            std::vector<mt::UInt> cur_data_vect;
            cur_lang_distr.copyToVector(cur_data_vect);
            st::Xi2Calculator x2Calculator(inp_data_vect, cur_data_vect);
            mt::Real x2_value = x2Calculator.getValue();
            if (x2_value < best_x2_value) {
                best_x2_value = x2_value;
                best_lang = static_cast<LanguageID>(lang_id);
            }
        }

        glob::assert(best_x2_value < xi_2_14_095);

        return best_lang;
    }

    //
    // Internal methods
    //   


} // namespace lang
                                                                                 