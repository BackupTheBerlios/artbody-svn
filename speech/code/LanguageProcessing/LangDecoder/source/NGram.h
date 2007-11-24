#ifndef __ngram_h__
#define __ngram_h__

#include <vector>
#include <map>
#include <math.h>

namespace st {

    static const float ZERO_PROBABILITY     = 0.000000001f;
    static const float ZERO_LOG_PROBABILITY = -1000.f;

    template<typename T>
    class BiGram {
    public:
        typedef T               ElemType;
        typedef std::vector<T>  DataVector;

        BiGram();

        bool train               (const DataVector& train_data);
        // normalize and calc probability logarithm
        void normalizeProbalility(void);

        float calcSequenceLogProbability      (const DataVector& input_sequence) const ;
        float getElemLogProbabilityWithHistory(const ElemType& elem, const DataVector& history) const;
    private:
        enum _SYS_ELEMS {
            _SYS_ELEMS_START = 0,
            _SYS_ELEMS_END,
            _SYS_ELEMS_MAX
        };

        std::map<ElemType, int>    m_elemIndexesMap;

        typedef std::vector<float> _FreqVector;
        std::vector<_FreqVector>   m_freqTable;

        int                        _fillElemMap           (const DataVector& train_data);
        void                       _calcFrequencies       (const DataVector& train_data);
        void                       _processOneTrainElem   (const ElemType& elem, int elem_index, const DataVector& train_data);        

        float                      _getStartEndProbability(const DataVector& input_sequence) const;
    };

    template<typename T>
    BiGram<T>::BiGram()
    {
        // init frequency table (with START and END elems)
        m_freqTable.resize(_SYS_ELEMS_MAX);
        for (int rowIdx = 0; rowIdx < _SYS_ELEMS_MAX; rowIdx++) {
            _FreqVector& tableRow = m_freqTable[rowIdx];
            tableRow.resize(_SYS_ELEMS_MAX);
        }
    }

    template<typename T>
    bool BiGram<T>::train(const DataVector& train_data)
    {
        // Correspond index value for each distinct element in data set
        int nDistValues = _fillElemMap(train_data);
        
        // adjust sizes of frequency table
        m_freqTable.resize(nDistValues);
        for (int rowIdx = 0; rowIdx < nDistValues; rowIdx++) {
            _FreqVector& tableRow = m_freqTable[rowIdx];
            tableRow.resize(nDistValues);
        }

        // calc frequesnces
        _calcFrequencies(train_data);

        return true;
    }

    template<typename T>
    void BiGram<T>::normalizeProbalility(void)
    {
        int freqTableSize = static_cast<int>(m_freqTable.size());
        for (int row_index = 0; row_index < freqTableSize; row_index++) {
            // calc summ of frequencies
            float row_summ = 0.f;
            for (int coll_index = 0; coll_index < freqTableSize; coll_index++) {
                row_summ += m_freqTable[row_index][coll_index];
            }
            // normalize
            for (int coll_index = 0; coll_index < freqTableSize; coll_index++) {
                float value = m_freqTable[row_index][coll_index] / row_summ;
                if (value > 0.00000001f) {
                    m_freqTable[row_index][coll_index] = log(value);
                } else {
                    m_freqTable[row_index][coll_index] = ZERO_LOG_PROBABILITY;
                }
            }
        }
    }

    template<typename T>
    float BiGram<T>::calcSequenceLogProbability(const DataVector& input_sequence) const
    {
        if (input_sequence.size() == 0) {
            return ZERO_LOG_PROBABILITY;
        }

        float seq_prob = 0.f;
        DataVector::const_iterator elem_iter = input_sequence.end();        
        elem_iter--;
        for (; elem_iter != input_sequence.begin(); elem_iter--) {
            DataVector::const_iterator history_end_iter = elem_iter;
            history_end_iter--;
            DataVector history(input_sequence.begin(), history_end_iter);
            seq_prob += getElemLogProbabilityWithHistory(*elem_iter, history);
        }

        // add probability of start and end of sequense
        seq_prob += _getStartEndProbability(input_sequence);

        return seq_prob;
    }

    template<typename T>
    float BiGram<T>::getElemLogProbabilityWithHistory(const ElemType& elem, const DataVector& history) const
    {
        if (history.size() == 0) {
            return ZERO_LOG_PROBABILITY;
        }

        std::map<ElemType, int>::const_iterator find_iter;
        
        // find index of 'elem'
        find_iter = m_elemIndexesMap.find(elem);
        if (find_iter == m_elemIndexesMap.end()) {
            return ZERO_LOG_PROBABILITY;
        }
        int rowIndex = find_iter->second;
        
        // find index of last in history
        find_iter = m_elemIndexesMap.find(history.back());
        if (find_iter == m_elemIndexesMap.end()) {
            return ZERO_LOG_PROBABILITY;
        }
        int collIndex = find_iter->second;

        return m_freqTable[rowIndex][collIndex];
    }

    //
    // Internal methods
    //
    template<typename T>
    int BiGram<T>::_fillElemMap(const DataVector& train_data)
    {
        int nDistValues = static_cast<int>(m_elemIndexesMap.size()) + _SYS_ELEMS_MAX;
        for (DataVector::const_iterator data_iter = train_data.begin(); data_iter != train_data.end(); data_iter++) {
            const ElemType& elem = *data_iter;
            if (m_elemIndexesMap.find(elem) != m_elemIndexesMap.end()) {
                // the new one
                m_elemIndexesMap[elem] = nDistValues++;
            }
        }
        
        return nDistValues;
    }
    
    template<typename T>
    void BiGram<T>::_calcFrequencies(const DataVector& train_data)
    {
        // calc frequencies 
        for (std::map<ElemType, int>::const_iterator elem_iter = m_elemIndexesMap.begin(); elem_iter != m_elemIndexesMap.end(); elem_iter++) {
            const ElemType& elem = elem_iter->first;
            int elem_index = elem_iter->second;
            _processOneTrainElem(elem, elem_index, train_data);
        }        
    }

    template<typename T>
    void BiGram<T>::_processOneTrainElem(const ElemType& elem, int elem_index, const DataVector& train_data)
    {
        DataVector::const_iterator prev_iter = train_data.end();
        for (DataVector::const_iterator data_iter = train_data.begin();              
             data_iter != train_data.end(); 
             prev_iter = data_iter++ ) {

            const ElemType& train_elem = *data_iter;
            if (train_elem == elem) { // let's look on prev 
                if (prev_iter != train_data.end()) {
                    int prev_index = m_elemIndexesMap[*prev_iter];
                    m_freqTable[elem_index][prev_index]++;
                } else { // first elem
                    m_freqTable[elem_index][_SYS_ELEMS_START]++;
                }// if
            } // if
        } // for
        // handle last elem
        int prev_index = m_elemIndexesMap[*prev_iter];
        m_freqTable[_SYS_ELEMS_END][prev_index]++;
    }    

    template<typename T>
    float BiGram<T>::_getStartEndProbability(const DataVector& input_sequence) const
    {
        std::map<ElemType, int>::const_iterator find_iter;
        
        // handle start
        // find index of first elem
        float startProb = 0.f;
        find_iter = m_elemIndexesMap.find(*input_sequence.begin());
        if (find_iter == m_elemIndexesMap.end()) {
            startProb = ZERO_LOG_PROBABILITY;
        } else {
            int rowIndex = find_iter->second;
            startProb = m_freqTable[rowIndex][_SYS_ELEMS_START];
        }

        // handle end
        float endProb = 0.f;
        find_iter = m_elemIndexesMap.find(*(--input_sequence.end()));
        if (find_iter == m_elemIndexesMap.end()) {
            endProb = ZERO_LOG_PROBABILITY;
        } else {
            int collIndex = find_iter->second;
            endProb = m_freqTable[_SYS_ELEMS_END][collIndex];
        }
        
        return startProb + endProb;
    }

} // namespace st

#endif //__ngram_h__