#ifndef __gaussdistr_h__
#define __gaussdistr_h__

#include <list>
#include <vector>

namespace st {
    template<typename T>
    class GaussDistribCalculator {
    public:
        typedef T                     ElemType;
        typedef std::list<ElemType>   ElemList;
        typedef std::vector<ElemType> ElemVector;

        GaussDistribCalculator();

        bool  calcParams(const ElemVector& input_data);

        float getMean     (void);
        float getVariance (void);
    private:
        float m_mean;
        float m_variance;
    };

    template<typename T>
    GaussDistribCalculator<T>::GaussDistribCalculator() :
    m_mean(0.f),
    m_variance(0.f)
    {        
    }

    template<typename T>
    float GaussDistribCalculator<T>::getMean(void)
    {
        return m_mean;
    }

    template<typename T>
    float GaussDistribCalculator<T>::getVariance(void)
    {
        return m_variance;
    }

    template<typename T>
    bool GaussDistribCalculator<T>::calcParams(const ElemVector& input_data)
    {
        ElemVector::const_iterator data_iter;

        // calc mean
        float sum = 0.f;
        int   nElem = 0;
        for (data_iter = input_data.begin(); data_iter != input_data.end(); data_iter++, nElem++) {
            sum += (float)(*data_iter);
        }

        m_mean = sum / nElem;

        // calc variance
        float squared_err = 0.f;
        for (data_iter = input_data.begin(); data_iter != input_data.end(); data_iter++) {
            float err = ((float)(*data_iter) - m_mean);
            squared_err += err * err;
        }

        m_variance = squared_err / nElem;

        return true;
    }

} // namespace st

#endif //__gaussdistr_h__