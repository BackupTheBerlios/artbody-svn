#ifndef __langdecoder_h__
#define __langdecoder_h__

#include <string>
#include <vector>

namespace lang {

enum LanguageID {
    Language_ENG,
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

class Decoder {
public:
    Decoder();

    bool       trainLanguage  (const std::string& file_in, LanguageID language);
    LanguageID resolveLanguage(const std::string& file_in);
private:
    struct _CharDescr {
        _CharDescr(unsigned char c, int i) { pair.first = c; pair.second = i; }
        std::pair<unsigned char, int> pair;
        bool operator <(_CharDescr& other) { return pair.second < other.pair.second; } 
    };

    class _CharFreqIndex {
    public:
        _CharFreqIndex();
        void putChar(unsigned char c);
        void sort   (void);

        int getCharIndex(unsigned char c);
    private:
        std::vector<_CharDescr> m_charFreq_index;               
    };

    struct _LangDescr {
        _CharFreqIndex          index;
        st::BiGram<int>         langModel;
    };
    _LangDescr m_trainedLanguages[Language_MAX];

    bool _read          (const std::string& file_in, std::vector<unsigned char>& input_data);
    void _fillFreqVector(_CharFreqIndex& index, const std::vector<unsigned char>& file_data);
};

} // namespace lang

#endif //__langdecoder_h__