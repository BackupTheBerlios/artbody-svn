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
            unsigned int n; // count of elements in initial data set with given frequency
            mt::Real Z;
            mt::Real logR;
            mt::Real logZ;
            mt::Real r_asterix; 
            mt::Real prob;
            _STGTableRow(): n(0), Z(0.0), logR(0.0), logZ(0.0), r_asterix(0.0), prob(0.0) {}
        };

    };
    
    template <typename T>
    SGTSmoother::smooth(DataProbVector& data_vector)
    {
        // calculate individual frequencies        
        std::map<ElemType, unsigned int> elemsMap;

        unsigned int data_size = data_vector.size();
        for (unsigned int data_iter = 0; data_iter < data_size; data_iter++) {
            ElemType& key = data_vector[data_iter].first;
            elemsMap[key] += 1;
        }
        
        // calculate count of different frequencies        
        std::map<unsigned int, _STGTableRow> frequencyMap;
        for (std::map<ElemProbEntity>::iterator elems_iter = elemsMap.begin(); elems_iter != elemsMap.end(); elems_iter++) {
            unsigned int elemFreq = *elems_iter.second;
            // will work with fresh ones and existing ones due to default constructor of _STGTableRow
            frequencyMap[elemFreq].n++;
        }

        unsigned int nDiffFreqs = frequencyMap.size();
        // calculate unsmoothed probability value
        for (std::map<unsigned int, _STGTableRow>::iterator freq_iter = frequencyMap.begin(); freq_iter != frequencyMap.end(); freq_iter++) {
            _STGTableRow& curRow = *freq_iter.second;
            unsigned int curFreq = *freq_iter.first;
            curRow.prob = static_cast<mt::Real>(curFreq) / data_size;
        }

        if (!_calcP0(frequencyMap, data_size)) {
            _copyProbabilities(frequencyMap, data_vector);
            return false;
        }

        if (!_calcZ(frequencyMap, data_size)) {
            _copyProbabilities(frequencyMap, data_vector);
            return false;
        }

        if (!_calcLogs(frequencyMap, data_size)) {
            _copyProbabilities(frequencyMap, data_vector);
            return false;
        }

        mt::Real a, b;
        if (!_calcAB(frequencyMap, data_size, a, b)) {
            _copyProbabilities(frequencyMap, data_vector);
            return false;
        }

        if (!_calcR_asterix(frequencyMap, data_size, a, b)) {
            _copyProbabilities(frequencyMap, data_vector);
            return false;
        }

        if (!_calcSmoothedProbabilities(frequencyMap, data_size)) {
            _copyProbabilities(frequencyMap, data_vector);
            return false;
        }

        _copyProbabilities(frequencyMap, data_vector);

        return true;
    }


} // namespace st

#endif //__stgsmoother_h__