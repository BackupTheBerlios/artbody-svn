#ifndef _AB_ADAPTER_H_
#define _AB_ADAPTER_H_

#include "ab_types.h"

// forvard
namespace inp {
struct InputParams;
}

namespace mrt {
class Vect3;
class Se3;
class Mat3;
}

namespace ab 
{

// forward
class Matr4;
class Vect4;

const DWord COLOR_RED   = 0xFFFF0000;
const DWord COLOR_GREEN = 0xFF00FF00;
const DWord COLOR_BLUE  = 0xFF0000FF;

enum ColorComponent {
    COLOR_A,
    COLOR_R,
    COLOR_G,
    COLOR_B
};

struct GLMatr {
    float elems[16];
};


//////////////////////////////////////////////////////////////////////////
// Adapter class
//////////////////////////////////////////////////////////////////////////
class Adapter 
{
public:
    static GLMatr matr4ToGl(const Matr4& matr_);

    static void Vect4ToMirt(const Vect4& srcVect, mrt::Vect3& dstVect);
    static void Se3ToMatr4 (const mrt::Se3& se3, Matr4& matr);
    static void Matr4ToSe3 (const Matr4& matr, mrt::Se3& se3);

    static void MirtToVect4(const mrt::Vect3& srcVect, Vect4& dstVect);
    static void MirtToMatr4(const mrt::Mat3& srcMatr, Matr4& dstMatr);
};

//////////////////////////////////////////////////////////////////////////
// Helper functions
//////////////////////////////////////////////////////////////////////////
inline Byte GetColorComp(DWord color, ColorComponent comp)
{
    return ((color >> ((COLOR_B - comp) * 8)) & 0xFF);
}


} // end of namespace ab

#endif //_ADAPTER_H_
