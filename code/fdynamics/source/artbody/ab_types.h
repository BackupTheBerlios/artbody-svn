#ifndef _AB_TYPES_H_
#define _AB_TYPES_H_

namespace ab 
{

// typedefs
typedef unsigned int  DWord;
typedef unsigned char Byte;
typedef double        Real;

const Real Epsilon = 0.000001f;
const Real PI = Real(3.14159265358979323846f);

inline Real DegToRad(Real argDeg) { return argDeg * (PI / 180.0); } 
inline Real RadToDeg(Real argRad) { return argRad * (180.0 / PI); } 

inline Real clampValue(Real leftBound, Real rightBound, Real value) 
{ 
    if (leftBound > rightBound) {
        // assert!!!!
        return value;
    } 

    if (value < leftBound) {
        return leftBound;
    }

    if (value > rightBound) {
        return rightBound;
    }

    return value;
}


} // end of namespace ab

#endif //_AB_TYPES_H_
