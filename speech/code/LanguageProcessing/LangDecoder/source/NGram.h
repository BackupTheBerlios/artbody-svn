#ifndef __ngram_h__
#define __ngram_h__

#include <vector>
#include <map>

namespace st {

    template<typename T>
    class BiGram {
    public:
        typedef T               ElemType;
        typedef std::vector<T>  DataVector;

        bool train(const DataVector& train_data);

        float calcSequenceLogPrbability(DataVector& input_sequence);
    private:
        std::map<ElemType, int>    m_elemMap;

        typedef std::vector<float> _FreqVector;
        std::vector<_FreqVector>   m_freqTable;

        int                        _fillElemMap         (const DataVector& train_data);
        void                       _calcFrequencies     (const DataVector& train_data);
        void                       _processOneTrainElem (const ElemType& elem, int elem_index, const DataVector& train_data);
        void                       _normalizeProbalility(void);
    };

    template<typename T>
    bool BiGram<T>::train(const DataVector& train_data)
    {
        // Correspond index value for each distinct element in data set
        int nDistValues = _fillElemMap(train_data);
        
        // adjust sizes of frequency table
        m_freqTable.resize(nDistValues);
        for (int strIdx = 0; strIdx < nDistValues; strIdx++) {
            _FreqVector& tableStr = m_freqTable[strIdx];
            tableStr.resize(nDistValues);
        }

        // calc frequesnces
        _calcFrequencies(train_data);

        // normalize and calc probability logarithm
        _normalizeProbalility();
    }

    template<typename T>
    float BiGram<T>::calcSequenceLogPrbability(DataVector& input_sequence)
    {
        
    }

    template<typename T>
    int BiGram<T>::_fillElemMap(const DataVector& train_data)
    {
        int nDistValues = m_elemMap.size();
        for (DataVector::const_iterator data_iter = train_data.begin(); data_iter != train_data.end(); data_iter++) {
            const ElemType& elem = *data_iter;
            if (m_elemMap.find(elem) != m_elemMap.end()) {
                // the new one
                m_elemMap[elem] = nDistValues;
                nDistValues++;
            }
        }
        
        return nDistValues;
    }
    
    template<typename T>
    void BiGram<T>::_calcFrequencies(const DataVector& train_data)
    {
        // calc frequencies 
        for (std::map<ElemType, int>::const_iterator elem_iter = m_elemMap().begin(); elem_iter != m_elemMap().end(); elem_iter++) {
            const ElemType& elem = elem_iter->first();
            int elem_index = elem_iter->second();
            _processOneTrainElem(elem, elem_index, train_data);
        }        
    }

    template<typename T>
    void BiGram<T>::_processOneTrainElem(const ElemType& elem, int elem_index, const DataVector& train_data)
    {
        for (DataVector::const_iterator data_iter = train_data.begin(), 
             DataVector::const_iterator prev_iter = train_data.end();
             data_iter != train_data.end(); 
             prev_iter = data_iter++ ) {

            const ElemType& train_elem = data_iter;
            if (train_elem == elem) { // let's look on prev 
                if (prev_iter != train_data.end()) {
                    int prev_index = m_elemMap[prev_iter];
                    m_freqTable[elem_index][prev_index]++;
                } // if

            } // if
        } // for
    }

    template<typename T>
    void BiGram<T>::_normalizeProbalility(void)
    {
        unsigned int freqTableSize = m_freqTable.size();
        for (int str_index = 0; str_index < freqTableSize; str_index++) {
            // calc summ of frequencies
            float str_summ = 0.f;
            for (coll_index = 0; coll_index < freqTableSize; coll_index++) {
                str_summ += m_freqTable[str_index][coll_index];
            }
            // normalize
            for (coll_index = 0; coll_index < freqTableSize; coll_index++) {
                float value = m_freqTable[str_index][coll_index] / str_summ;
                if (value > 0.00000001f) {
                    m_freqTable[str_index][coll_index] = log(value);
                } else {
                    m_freqTable[str_index][coll_index] = 0.f;
                }
            }
        }
    }

} // namespace st

#endif //__ngram_h__