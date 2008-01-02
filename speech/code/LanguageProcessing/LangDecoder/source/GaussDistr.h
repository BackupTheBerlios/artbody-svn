#ifndef __gaussdistr_h__
#define __gaussdistr_h__

#include <list>
#include <vector>

#include "mt.h"

namespace st {
    template<typename T>
    class GaussDistribCalculator {
    public:
        typedef T                     ElemType;
        typedef std::list<ElemType>   ElemList;
        typedef std::vector<ElemType> ElemVector;

        GaussDistribCalculator();

        bool  calcParams(const ElemVector& input_data);

        mt::Real getMean       (void) { return m_mean; }
        mt::Real getVariance   (void) { return m_variance; }
        mt::Real getSelVariance(void) { return m_sel_variance; }
    private:
        mt::Real m_mean;
        mt::Real m_variance;
        mt::Real m_sel_variance;
    };

    template<typename T>
    GaussDistribCalculator<T>::GaussDistribCalculator() :
    m_mean(0.0),
    m_variance(0.0),
    m_sel_variance(0.0)
    {        
    }

    template<typename T>
    bool GaussDistribCalculator<T>::calcParams(const ElemVector& input_data)
    {
        ElemVector::const_iterator data_iter;

        // calc mean
        mt::Real sum = 0.f;
        int   nElem = 0;
        for (data_iter = input_data.begin(); data_iter != input_data.end(); data_iter++, nElem++) {
            sum += (mt::Real)(*data_iter);
        }

        m_mean = sum / nElem;

        // calc variance
        mt::Real squared_err = 0.f;
        for (data_iter = input_data.begin(); data_iter != input_data.end(); data_iter++) {
            mt::Real err = ((mt::Real)(*data_iter) - m_mean);
            squared_err += err * err;
        }

        m_variance = squared_err / nElem;

        m_sel_variance = squared_err / (nElem - 1);

        return true;
    }

} // namespace st

#endif //__gaussdistr_h__