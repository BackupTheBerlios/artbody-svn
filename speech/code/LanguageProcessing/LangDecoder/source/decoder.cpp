#include <stdio.h>
#include <string>

#include <algorithm>

#include "glob.h"
#include "LangDecoder.h"
#include "DecoderBiGram.h"
#include "DecoderVariance.h"
#include "DecoderSpaceFreq.h"

namespace lang {

//
// class Decoder
//
Decoder::Decoder(DecoderType type) 
{
    switch (type) {
        case DecoderTypeBiGram:
            m_pImpl = new BiGramDecoder;
            break;
        case DecoderTypeVariance:
            m_pImpl = new VarianceDecoder;
            break;
        case DecoderTypeSpaceFreq:
            m_pImpl = new SpaceFreqDecoder;
            break;
    }
}

Decoder::~Decoder()
{
    delete m_pImpl;
    m_pImpl = NULL;
}

//
// class DecoderImpl
// 
bool DecoderImpl::_read(const std::string& file_in, unsigned char*& file_data, unsigned int& data_len)
    {
        // open file
        FILE* fp = fopen(file_in.c_str(), "rb");
        if (fp == NULL) {
            return false;
        }

        // calc size of file
        fseek(fp, 0, SEEK_END);
        unsigned int fileSize = ftell(fp);
        fseek(fp, 0, SEEK_SET);

        // alloc storage vector
        data_len = fileSize / sizeof(char);
        file_data = new unsigned char[data_len * sizeof(char)];
        if (file_data == NULL) {
            return false;
        }
        
        // read data into storage vector
        unsigned int nChars = 0;
        while (!feof(fp) && (nChars < data_len)){
            unsigned char cur_c = static_cast<unsigned char>(fgetc(fp));    
            file_data[nChars++] = cur_c;
        }

        fclose(fp);

        return true;
    }

} // namespace lang
                                                                                 