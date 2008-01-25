#ifndef __stgsmoother_h__
#define __stgsmoother_h__

#include "mt.h"

namespace st {
    template <typename T>
    class SGTSmoother {        
    public:
        typedef T ElemType;
        typedef std::pair<ElemType, mt::Real> ElemProbEntity;

        typedef std::vector<ElemProbEntity> DataProbVector;

        SGTSmoother(void);

        bool smooth(DataProbVector& data_vector);
    private:
        struct _STGTableRow {            
            mt::UInt frequency; 
            mt::UInt n; // count of elements in initial data set with given frequency
            mt::Real Z;
            mt::Real logR;
            mt::Real logZ;
            mt::Real r_asterix; 
            mt::Real prob;
            _STGTableRow(): frequency(0), n(0), Z(0.0), logR(0.0), logZ(0.0), r_asterix(0.0), prob(0.0) {}
        };

        typedef std::vector<mt::UInt, _STGTableRow> _STGTable;

        bool _calcP0                   (_STGTable& freqMap, mt::UInt all_data_size, mt::Real& P0);
        bool _calcZ                    (_STGTable& freqMap, mt::UInt all_data_size);
        bool _calcLogs                 (_STGTable& freqMap, mt::UInt all_data_size);
        bool _calcAB                   (_STGTable& frequencyMap, mt::UInt data_size, mt::Real& a, mt::Real& b);
        bool _calcR_asterix            (_STGTable& frequencyMap, mt::UInt data_size, mt::Real a, mt::Real b);
        bool _calcSmoothedProbabilities(_STGTable& freqMap, mt::UInt all_data_size);

        void _copyProbabilities(_STGTable& frequencyMap, std::map<ElemType, mt::UInt>& elemsMap, DataProbVector& data_vector);
    };
    
    template <typename T>
    SGTSmoother::smooth(DataProbVector& data_vector)
    {
        // calculate individual frequencies        
        std::map<ElemType, mt::UInt> elemsMap;

        mt::UInt data_size = data_vector.size();
        for (mt::UInt data_iter = 0; data_iter < data_size; data_iter++) {
            ElemType& key = data_vector[data_iter].first;
            elemsMap[key] += 1;
        }
        
        // calculate count of different frequencies        
        _STGTable frequencyMap;
        for (std::map<ElemProbEntity>::iterator elems_iter = elemsMap.begin(); elems_iter != elemsMap.end(); elems_iter++) {
            mt::UInt elemFreq = *elems_iter.second;
            // will work with fresh ones and existing ones due to default constructor of _STGTableRow
            
        }
        
        mt::UInt nDiffFreqs = frequencyMap.size();
        // calculate unsmoothed probability value
        for (_STGTable::iterator freq_iter = frequencyMap.begin(); freq_iter != frequencyMap.end(); freq_iter++) {
            _STGTableRow& curRow = *freq_iter;
            mt::UInt curFreq = curRow.frequency;
            curRow.prob = static_cast<mt::Real>(curFreq) / data_size;
        }
        
        mt::Real P0;
        if (!_calcP0(frequencyMap, data_size, P0)) {
            _copyProbabilities(frequencyMap, elemsMap, data_vector);
            return false;
        }

        if (!_calcZ(frequencyMap, data_size)) {
            _copyProbabilities(frequencyMap, elemsMap, data_vector);
            return false;
        }

        if (!_calcLogs(frequencyMap, data_size)) {
            _copyProbabilities(frequencyMap, elemsMap, data_vector);
            return false;
        }

        mt::Real a, b;
        if (!_calcAB(frequencyMap, data_size, a, b)) {
            _copyProbabilities(frequencyMap, elemsMap, data_vector);
            return false;
        }

        if (!_calcR_asterix(frequencyMap, data_size, a, b)) {
            _copyProbabilities(frequencyMap, elemsMap, data_vector);
            return false;
        }

        if (!_calcSmoothedProbabilities(frequencyMap, data_size)) {
            _copyProbabilities(frequencyMap, elemsMap, data_vector);
            return false;
        }

        _copyProbabilities(frequencyMap, elemsMap, data_vector);

        return true;
    }

    //
    // Internal methods
    //
    template<typename T>
    void SGTSmoother::_copyProbabilities(_FreqMap& frequencyMap, std::map<ElemType, mt::UInt>& elemsMap, DataProbVector& data_vector)
    {
        mt::UInt init_data_size = data_vector.size();
        for (mt::UInt data_iter = 0; data_iter < init_data_size; data_iter++) {            
            ElemType& elem = data_vector[data_iter].first;

            mt::UInt elem_freq = elemsMap[elem];

            _STGTableRow& elem_freq_info = *frequencyMap.find(elem_freq);
            
            data_vector[data_iter].second = elem_freq_info.prob;
        }

    }

    template<typename T>
    bool SGTSmoother::_calcP0(_FreqMap& freqMap, mt::UInt all_data_size, mt::Real& P0)
    {
        _FreqMap::iterator find_iter = freqMap.find(1);

        if (find_iter == freqMap.end()) {
            // there aren't elements in initial data set with frequency == 1
            return false;
        }

        _STGTableRow& first_row = *find_iter;
        P0 = static_cast<mt::Real>(first_row.n) / all_data_size;
        return true;
    }

    template<typename T>
    bool SGTSmoother::_calcZ(_FreqMap& freqMap, mt::UInt all_data_size)
    {


    }


} // namespace st

#endif //__stgsmoother_h__