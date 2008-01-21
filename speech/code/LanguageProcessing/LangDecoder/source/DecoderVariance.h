#ifndef __decodervariance_h__
#define __decodervariance_h__

#include "LangDecoder.h"
#include "GaussDistr.h"
#include "mt.h"

namespace lang {

class VarianceDecoder : public DecoderImpl {
public:
    VarianceDecoder();

    bool       trainLanguage  (const std::string& file_in, LanguageID language);
    LanguageID resolveLanguage(const std::string& file_in);
private:
    struct _LangDistr {
        bool init(const std::string& file_in);
        st::GaussDistribCalculator<mt::Real> distrib;

        // debug
        void _writeFreq(const std::string& file_in, std::vector<mt::Real>& freq_vect);
    };

    _LangDistr m_trainedLanguages[Language_MAX];    

    static mt::Real z_distr_255_255_a_005;
};

} // namespace lang

#endif //__decodervariance_h__
