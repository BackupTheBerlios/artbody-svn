#ifndef __decoderspacefreq_h__
#define __decoderspacefreq_h__

#include "LangDecoder.h"
#include "mt.h"

namespace lang {

class SpaceFreqDecoder : public DecoderImpl {
public:
    SpaceFreqDecoder();

    bool       trainLanguage  (const std::string& file_in, LanguageID language);
    LanguageID resolveLanguage(const std::string& file_in);
private:
    struct _LangDistr {
        _LangDistr() : nTotalWords(0) { memset(nWordsInGroup, 0, sizeof(nWordsInGroup)); } 
        bool init(const std::string& file_in);

        void copyToVector(std::vector<mt::UInt>& group_vector);
        
        static const unsigned int MAX_WORD_LEN = 15;

        unsigned int nWordsInGroup[MAX_WORD_LEN];
        unsigned int nTotalWords;
    private:
        unsigned char _detectSpaceChar(unsigned char* file_data, unsigned int data_len);
        void          _detectWords    (unsigned char* file_data, unsigned int data_len, unsigned char space_char);
    };

    _LangDistr m_trainedLanguages[Language_MAX];    

    static mt::Real xi_2_14_095;
};

} // namespace lang

#endif //__decoderspacefreq_h__