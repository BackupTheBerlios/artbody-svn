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
    };

    _LangDistr m_trainedLanguages[Language_MAX];    
};

} // namespace lang

#endif //__decodervariance_h__