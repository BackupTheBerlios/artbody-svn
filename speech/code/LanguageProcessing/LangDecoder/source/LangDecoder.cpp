#include <stdio.h>
#include <string>

#include "LangDecoder.h"

namespace glob {
    void assert(bool exp) 
    {
        if (!exp) {
            __asm int 3;
        }
    }
}

bool testEncode()
{
    lang::Encoder encoder;
    encoder.encodeFile("file_rus1.txt", "file_rus1_c.txt");
    encoder.encodeFile("file_eng1.txt", "file_eng1_c.txt");
    return true;
}

bool testDecode1()
{
    lang::Decoder decoder;
    decoder.trainLanguage("file_rus2.txt", lang::Language_RUS);
    decoder.trainLanguage("file_eng2.txt", lang::Language_ENG);
    return true;
}

//bool testAll(void) 
//{
//    lang::Encoder encoder;
//    lang::Decoder decoder;
//
//    std::string train_file_eng;
//    std::string train_file_rus;
//    decoder.trainLanguage(train_file_eng, lang::Language_END);
//    decoder.trainLanguage(train_file_rus, lang::Language_RUS);
//
//    std::string file_in_1, file_out_1;
//    std::string file_in_2, file_out_2;
//    encoder.encodeFile(file_in_1, file_out_1);
//    encoder.encodeFile(file_in_2, file_out_2);
//    lang::LanguageID language_1 = decoder.resolveLanguage(file_out_1);
//    lang::LanguageID language_2 = decoder.resolveLanguage(file_out_2);
//
//    bool rc1 = (language_1 == lang::Language_END);
//    bool rc2 = (language_2 == lang::Language_RUS);
//    glob::assert(rc1);
//    glob::assert(rc2);
//
//    return rc1 && rc2;
//}

int main(int argc, char* argv[])
{
    testEncode();
    testDecode1();

    return 0;
}