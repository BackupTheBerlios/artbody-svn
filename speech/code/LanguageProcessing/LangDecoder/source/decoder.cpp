#include <stdio.h>
#include <string>

#include "LangDecoder.h"
#include "GaussDistr.h"
#include "NGram.h"

namespace lang {

    Decoder::Decoder()
    {
        ;
    }

    bool Decoder::trainLanguage(const std::string& file_in, LanguageID language)
    {
        std::vector<unsigned char> file_data;
        // read file data 
        if (!_read(file_in, file_data)) {
            return false;
        }        
        
        std::vector<_CharDescr> freqVector;
        _fillFreqVector(freqVector, file_data);



        return true;
    }

    bool Decoder::_read(const std::string& file_in, std::vector<unsigned char>& input_data)
    {
        // open file
        FILE* fp = fopen(file_in.c_str(), "r");
        if (fp == NULL) {
            return false;
        }

        // calc size of file
        fseek(fp, 0, SEEK_END);
        unsigned int fileSize = ftell(fp);
        fseek(fp, 0, SEEK_SET);

        // realloc storage vector
        unsigned int nElems = fileSize / sizeof(char);
        input_data.resize(nElems);
        
        // read data into storage vector
        unsigned int nChars = 0;
        while (!feof(fp)){
            unsigned char cur_c = static_cast<unsigned char>(fgetc(fp));    
            input_data[nChars] = cur_c;
            nChars++;
        }

        fclose(fp);
        
        return true;
    }

    void Decoder::_fillFreqVector(std::vector<_CharDescr>& freqVector, const std::vector<unsigned char>& file_data)
    {
        // init


        // calc frequencies
        for (std::vector<unsigned char>::const_iterator data_iter = file_data.begin(); data_iter != file_data.end(); data_iter++) {
            
        }

        // sort
    }

} // namespace lang
                                                                                 