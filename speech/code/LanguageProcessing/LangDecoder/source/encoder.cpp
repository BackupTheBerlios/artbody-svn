#include <stdio.h>
#include <string>

#include "LangDecoder.h"

namespace lang {

    Encoder::Encoder():
    m_file_size(0),
    m_file_buff(NULL)
    {
        ;
    }

    bool Encoder::encodeFile(const std::string& file_in, const std::string& file_out)
    {
#define HANDLE_ENCODING_ERROR(rc)     \
        if (!rc) {          \
            _freeBuffer();  \
            return false;   \
        }

        bool rc = _read(file_in);
        HANDLE_ENCODING_ERROR(rc);
        rc = _encodeBuffer();
        HANDLE_ENCODING_ERROR(rc);
        rc = _write(file_out);
        HANDLE_ENCODING_ERROR(rc);
#undef HANDLE_ENCODING_ERROR
        
        _freeBuffer();
        return true;
    }

    //
    // Internal methods
    //
    bool Encoder::_read(const std::string& file_in)
    {
        if (m_file_buff) {
            return false;
        }

        FILE* fp = fopen(file_in.c_str(), "r");
        if (fp == NULL) {
            return false;
        }
        fseek(fp, 0, SEEK_END);
        m_file_size = ftell(fp);
        fseek(fp, 0, SEEK_SET);

        m_file_buff = new unsigned char[m_file_size];
        if (m_file_buff == NULL) {
            m_file_size = 0;
            fclose(fp);
            return false;
        }

        fread(m_file_buff, 1, m_file_size, fp);
        fclose(fp);
        
        return true;
    }

    bool Encoder::_encodeBuffer()
    {
        for (unsigned int char_pos = 0; char_pos < m_file_size; char_pos++) {
            unsigned char prev_value = m_file_buff[char_pos];
            unsigned char* p_char = m_file_buff + char_pos;

            *p_char = prev_value ^ 0x55;
        }

        return true;
    }

    bool Encoder::_write(const std::string& file_out)
    {
        FILE* fp = fopen(file_out.c_str(), "w");
        if (fp == NULL) {
            return false;
        }       

        fwrite(m_file_buff, 1, m_file_size, fp);
        fclose(fp);
        return true;
    }

    void Encoder::_freeBuffer(void)
    {
        if (m_file_buff) {
            delete [] m_file_buff;            
        }
        m_file_buff = NULL;
        m_file_size = 0;
    }

} // namespace lang
