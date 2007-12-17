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



} // namespace mt

#endif //__mt_h__