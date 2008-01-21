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

    };
    
    template <typename T>
    SGTSmoother::smooth(DataProbVector& data_vector)
    {
        // calculate individual frequencies        
        std::map<ElemProbEntity> elemsMap;

        unsigned int data_size = data_vector.size();
        for (unsigned int data_iter = 0; data_iter < data_size; data_iter++) {
            ElemType& key = data_vector[data_iter].first;
            elemsMap[key] += 1.0;
        }

        for (std::map<ElemProbEntity>::iterator map_iter = elemsMap.begin(); map_iter != elemsMap.end(); map_iter++) {
            *map_iter.second /= data_size;
        }


    }


} // namespace st

#endif //__stgsmoother_h__