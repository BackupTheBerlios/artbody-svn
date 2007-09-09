// This system
#include "ab_vect4.h"
#include "ab_matr4.h"
#include "ab_adapter.h"
#include "core/Vect3.h"
#include "core/Mat3.h"
#include "core/Se3.h"

using namespace ab;

void Adapter::Vect4ToMirt(const Vect4& srcVect, mrt::Vect3& dstVect)
{
  dstVect.x = srcVect.x;
  dstVect.y = srcVect.y;
  dstVect.z = srcVect.z;
}

void Adapter::MirtToVect4(const mrt::Vect3& srcVect, Vect4& dstVect)
{
  dstVect.x = srcVect.x;
  dstVect.y = srcVect.y;
  dstVect.z = srcVect.z;
}

void Adapter::MirtToMatr4(const mrt::Mat3& srcMatr, Matr4& dstMatr)
{
  mrt::Vect3 mrt_axis;
  Vect4 la_axis;

  mrt_axis = srcMatr.xcol();
  MirtToVect4(mrt_axis, la_axis);
  dstMatr.setAxisX(la_axis);

  mrt_axis = srcMatr.ycol();
  MirtToVect4(mrt_axis, la_axis);
  dstMatr.setAxisY(la_axis);

  mrt_axis = srcMatr.zcol();
  MirtToVect4(mrt_axis, la_axis);
  dstMatr.setAxisZ(la_axis);
}

void Adapter::Se3ToMatr4(const mrt::Se3& se3, Matr4& matr)
{
  mrt::Vect3 tr = se3.trans();   
  mrt::Mat3 rot(se3.rot());

  Vect4 trans;
  Matr4 rotation;
  MirtToVect4(tr, trans);
  MirtToMatr4(rot, rotation);

  matr = rotation;
  matr.setTranslate(trans);
}

GLMatr Adapter::matr4ToGl(const Matr4& matr_)
{
    GLMatr glMatr;
    int i, j;

    for (j = 0; j < 4; j++) {
        for (i = 0; i < 4; i++) {
            glMatr.elems[j * 4 + i] = matr_.elem(i, j);
        }
    }
    return glMatr;
}
