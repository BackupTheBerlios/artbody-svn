#ifndef __decodercharsfreq_h__
#define __decodercharsfreq_h__

#include <vector>

#include "LangDecoder.h"
#include "mt.h"

namespace lang {

class CharsFreqDecoder : public DecoderImpl {
public:
    CharsFreqDecoder();

    bool       trainLanguage  (const std::string& file_in, LanguageID language);
    LanguageID resolveLanguage(const std::string& file_in);
private:
    struct _LangDistr {
        _LangDistr() : charsFreqVector(256, 0) { ; } 
        bool init(const std::string& file_in);

        std::vector<mt::UInt> charsFreqVector;
    private:
        unsigned char _detectSpaceChar(unsigned char* file_data, mt::UInt data_len);
    };

    _LangDistr m_trainedLanguages[Language_MAX];    

    static mt::Real xi_2_255_095;
};

} // namespace lang

#endif //__decodercharsfreq_h__