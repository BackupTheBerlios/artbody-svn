#ifndef __langdecoder_h__
#define __langdecoder_h__

#include <string>
#include <vector>

#include "NGram.h"

namespace lang {

enum LanguageID {
    Language_ENG = 0,
    Language_RUS,
    Language_MAX
};

class Encoder {
public:
    Encoder();

    bool encodeFile(const std::string& file_in, const std::string& file_out);
private:
    unsigned int   m_file_size;
    unsigned char* m_file_buff;

    bool _read        (const std::string& file_in);
    bool _encodeBuffer(void);
    bool _write       (const std::string& file_out);

    void _freeBuffer(void);
};

enum DecoderType {
    DecoderTypeBiGram
    ,DecoderTypeVariance
    ,DecoderTypeSpaceFreq
};

class DecoderImpl {
public:
    virtual bool       trainLanguage  (const std::string& file_in, LanguageID language) = 0;
    virtual LanguageID resolveLanguage(const std::string& file_in) = 0;
protected:
    static bool _read (const std::string& file_in, unsigned char*& file_data, unsigned int& data_len);
};

class Decoder {
public:
    explicit Decoder(DecoderType type);
    virtual ~Decoder();

    bool       trainLanguage  (const std::string& file_in, LanguageID language) { return m_pImpl->trainLanguage(file_in, language); }
    LanguageID resolveLanguage(const std::string& file_in) { return m_pImpl->resolveLanguage(file_in); }
private:
    DecoderImpl* m_pImpl;
};

} // namespace lang

#endif //__langdecoder_h__