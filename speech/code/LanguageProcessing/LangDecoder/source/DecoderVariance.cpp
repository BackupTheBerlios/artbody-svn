#include <stdio.h>
#include <string>

#include <vector>

#include "glob.h"
#include "DecoderVariance.h"

namespace lang {

    mt::Real VarianceDecoder::z_distr_255_255_a_005 = 0.08275723;
    //
    // struct VarianceDecoder 
    //
    bool VarianceDecoder::_LangDistr::init(const std::string& file_in)
    {
        unsigned char* file_data = NULL;
        unsigned int data_len = 0;
        // read file data 
        if (!VarianceDecoder::_read(file_in, file_data, data_len)) {
            return false;
        }

        std::vector<mt::Real> freq_vect(256, 0.0);
        for (unsigned int inp_iter = 0; inp_iter < data_len; inp_iter++) {
            unsigned char cur_char = file_data[inp_iter];
            freq_vect[cur_char] += 1.0;
        }

        for (unsigned int char_iter = 0; char_iter < freq_vect.size(); char_iter++) {
            freq_vect[char_iter] /= data_len;
        }
                
        distrib.calcParams(freq_vect);

        // debug
        _writeFreq(file_in, freq_vect);
        // end debug

        delete [] file_data;

       return true;
    }

    void VarianceDecoder::_LangDistr::_writeFreq(const std::string& file_in, std::vector<mt::Real>& freq_vect)
    {
        std::string freq_file_name(file_in);
        freq_file_name.insert(freq_file_name.find_last_of('.'), "_freq");

        FILE* fp = fopen(freq_file_name.c_str(), "w");
        if (fp == NULL) {
            return;
        }       
        for (unsigned int char_iter = 0; char_iter < freq_vect.size(); char_iter++) {
            fprintf(fp, "%f\n", freq_vect[char_iter]);            
        }
        fclose(fp);
        return;
    }


    //
    // Class lang::VarianceDecoder
    //
    VarianceDecoder::VarianceDecoder()
    {
        ;
    }

    bool VarianceDecoder::trainLanguage(const std::string& file_in, LanguageID language)
    {
        _LangDistr& inp_lang_distr = m_trainedLanguages[language];        
        return inp_lang_distr.init(file_in);
    }

    LanguageID VarianceDecoder::resolveLanguage(const std::string& file_in)
    {
        _LangDistr inp_lang_distr;
        inp_lang_distr.init(file_in);
        mt::Real inp_sel_variance = inp_lang_distr.distrib.getSelVariance();

        LanguageID best_lang = Language_ENG;

        //mt::Real bestVarianceDiff = mt::INFINITY;
        //for (int lang_id = 0; lang_id < Language_MAX; lang_id++) {
        //    _LangDistr& cur_lang_distr = m_trainedLanguages[lang_id];
        //    mt::Real cur_lang_sel_variance = cur_lang_distr.distrib.getSelVariance();
        //    mt::Real curVarianceDiff = abs(cur_lang_sel_variance - inp_sel_variance);
        //    if (curVarianceDiff < bestVarianceDiff) {
        //        best_lang = static_cast<LanguageID>(lang_id);
        //        bestVarianceDiff = curVarianceDiff;
        //    }
        //}

        mt::Real best_ln_var_ratio = mt::INFINITY;
        for (int lang_id = 0; lang_id < Language_MAX; lang_id++) {
            _LangDistr& cur_lang_distr = m_trainedLanguages[lang_id];
            mt::Real cur_lang_sel_variance = cur_lang_distr.distrib.getSelVariance();

            mt::Real cur_ln_var_ratio = abs(log(cur_lang_sel_variance / inp_sel_variance));
            if (cur_ln_var_ratio < best_ln_var_ratio) {
                best_lang = static_cast<LanguageID>(lang_id);
                best_ln_var_ratio = cur_ln_var_ratio;
            }            
        }

        glob::assert(best_ln_var_ratio < z_distr_255_255_a_005);

        return best_lang;
    }

    //
    // Internal methods
    //   


} // namespace lang
                                                                                 
