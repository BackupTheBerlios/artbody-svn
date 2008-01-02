#ifndef __mt_h__
#define __mt_h__

#include <math.h>

namespace mt {

    typedef double Real;

    static const Real EPSILON = 1e-16;
    static const Real INFINITY = 1e+16;

    inline bool isEqual(Real a, Real b)
    {
        return abs(a - b) < EPSILON;
    }

    inline mt::Real clamp(const mt::Real& value, const mt::Real& a, const mt::Real& b)
    {
        if (value < a) {
            return a;
        }
        if (value > b) {
            return b;
        }
        return value;
    }



} // namespace mt

#endif //__mt_h__