#ifndef __decoderbigram_h__
#define __decoderbigram_h__

#include <string>
#include <vector>

#include "NGram.h"
#include "LangDecoder.h"

namespace lang {

class BiGramDecoder : public DecoderImpl {
public:
    BiGramDecoder();

    bool       trainLanguage  (const std::string& file_in, LanguageID language);
    LanguageID resolveLanguage(const std::string& file_in);
private:
    struct _CharDescr {
        _CharDescr(unsigned char c = 0, int i = 0) { pair.first = c; pair.second = i; }
        std::pair<unsigned char, int> pair;
        
        static bool compareByFreq(_CharDescr& a, _CharDescr& b)   { return a.pair.second > b.pair.second; }
        static bool compareByChar(_CharDescr& a, _CharDescr& b)   { return bsCompareByChar(&a, &b) < 0; }
        static int  bsCompareByChar(_CharDescr* a, _CharDescr* b) { return a->pair.first - b->pair.first; }
        bool operator == (const _CharDescr& other) { return pair.first == other.pair.first; }
    };

    class _CharFreqIndex {
    public:
        _CharFreqIndex();
        void putChar(unsigned char c);
        void sort   (void);

        int           getCharIndex(unsigned char c);
        unsigned char getIndexChar(int index) const { return m_charFreq_index[index].pair.first; }
    private:
        std::vector<_CharDescr> m_charFreq_index;               
    };

    struct _LangDescr {
        _CharFreqIndex          index;
        st::BiGram<int>         langModel;
    };
    _LangDescr m_trainedLanguages[Language_MAX];

    void        _fillFreqVector              (_CharFreqIndex& index, const unsigned char* file_data, unsigned int data_len);
    void        _trainLanguageModel          (_LangDescr& lang_descr, const unsigned char* file_data, unsigned int data_len);
    mt::Real    _calcInputProbabilityWithLang(st::BiGram<int>& testModel, _CharFreqIndex& encodedIndex, const unsigned char* file_data, unsigned int data_len);
};

} // namespace lang

#endif //__decoderbigram_h__