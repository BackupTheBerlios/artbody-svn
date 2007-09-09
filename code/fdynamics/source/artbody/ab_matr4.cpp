// standard
#include <memory.h>
#include <math.h>
// this system
#include "ab_matr4.h"

namespace ab
{

Matr4 Matr4::Identity;

Matr4::Matr4(void)
{
   memset(_elems, 0, sizeof(_elems));
   _elems[0][0] = _elems[1][1] = _elems[2][2] = _elems[3][3] = 1.f;
}

Matr4::Matr4(const Matr4& matr_)
{
   memcpy(_elems, matr_._elems, sizeof(_elems));
}

Matr4::Matr4(const Vect4& pos)
{
  *this = Identity;
  setTranslate(pos);
}

Matr4::Matr4(const Vect4& rotVect, const Vect4& transVect, const Vect4& scaleVect)
{
   memset(_elems, 0, sizeof(_elems));
   setTranslate(transVect);

   rotateX(rotVect[0]);
   rotateY(rotVect[1]);
   rotateZ(rotVect[2]);

   scale(scaleVect);
}

Matr4::Matr4(const Vect4& xAxis, const Vect4& yAxis, const Vect4& zAxis, const Vect4& translate)
{
   memset(_elems, 0, sizeof(_elems));
   setAxisX(xAxis);
   setAxisY(yAxis);
   setAxisZ(zAxis);
   setTranslate(translate);
}

void Matr4::operator = (const Matr4& matr_)
{
   memcpy(_elems, matr_._elems, sizeof(_elems));
}

Matr4 Matr4::operator + (const Matr4& matr_) const
{
   Matr4 sum;
   int i, j;

   for (i = 0; i < 4; i++) {
      for (j = 0; j < 4; j++) {
         sum._elems[i][j] = _elems[i][j] + matr_._elems[i][j];
      }
   }

   sum._elems[3][3] = 1;

   return sum;
}

Matr4 Matr4::operator - (const Matr4& matr_) const
{
   Matr4 diff;
   int i, j;

   for (i = 0; i < 4; i++) {
      for (j = 0; j < 4; j++) {
         diff._elems[i][j] = _elems[i][j] - matr_._elems[i][j];
      }
   }

   diff._elems[3][3] = 1;

   return diff;
}

Matr4 Matr4::operator * (const Matr4& matr_) const
{   
   Matr4 mult;
   int i, row, col;

   for (row = 0; row < 4; row++ )  {
      for (col = 0; col < 4; col++ )  {
         mult._elems[row][col] = 0.f;
         for (i = 0; i < 4; i++ )  {
            mult._elems[row][col] += _elems[row][i] * matr_._elems[i][col];
         }
      }
   }

   mult._elems[3][3] = 1;
   
   return mult;
}

Vect4 Matr4::operator * (const Vect4& vect_) const
{
   Vect4 res;
   int i, j;

   for (i = 0; i < 4; i++) {
      for (j = 0; j < 4; j++) {
         res[i] += _elems[i][j] * vect_[j];
      }
   }

   return res; 
}

Vect4  Matr4::transformAsVector(const Vect4& vect_) const
{
    Vect4 res;
    int i, j;

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            res[i] += _elems[i][j] * vect_[j];
        }
    }

    return res; 
}

Vect4  Matr4::transformAsPoint(const Vect4& vect_) const
{
    return (*this) * vect_; 
}


void Matr4::transform(const Matr4& matr_, bool preTransform)
{
   Matr4 result;

   if (preTransform) {
      result = matr_ * (*this);      
   } else {
      result = (*this) * matr_;
   }

   memcpy(_elems, result._elems, sizeof(_elems));
}

void Matr4::translate(const Vect4& trans, bool preTransform)
{
   int i;

   if (preTransform) {
      for (i = 0; i < 3; i++) {
         _elems[i][3] += trans[i];
      }
   } else {
      for (i = 0; i < 3; i++) {
         _elems[i][3] += trans[0]*_elems[i][0] + trans[1]*_elems[i][1] + trans[2]*_elems[i][2];
      }
   }
   
   return;
}

void Matr4::translateLocal(AxisType dir, Real trans, bool preTransform)
{
   Vect4 axis;
   switch (dir) {
      case Axis_X:
         axis = getAxisX();
   	   break;
      case Axis_Y:
         axis = getAxisY();
         break;
      case Axis_Z:
         axis = getAxisZ();
         break;
   }
   axis.scale(trans);
   translate(axis, preTransform);

   return;
}

void Matr4::scale(const Vect4& sc, bool preTransform)
{
   int i;

   if (preTransform) {
      for (i = 0; i < 3; i++) {
         _elems[0][i] *= sc[i];
         _elems[1][i] *= sc[i];
         _elems[2][i] *= sc[i];
      }
   } else {
      for (i = 0; i < 3; i++) {
         _elems[i][0] *= sc[i];
         _elems[i][1] *= sc[i];
         _elems[i][2] *= sc[i];
      }
   }
   return;
}

void Matr4::rotateX(Real rotateAngleRad, bool preTransform)
{
   Matr4 rotMatr, resMatr;
   Real sinA = sin(rotateAngleRad), cosA = cos(rotateAngleRad);

   rotMatr._elems[1][1] = cosA;
   rotMatr._elems[1][2] = -sinA;
   rotMatr._elems[2][1] = sinA;
   rotMatr._elems[2][2] = cosA;

   transform(rotMatr, preTransform);
}

void Matr4::rotateY(Real rotateAngleRad, bool preTransform)
{
   Matr4 rotMatr, resMatr;
   Real sinA = sin(rotateAngleRad), cosA = cos(rotateAngleRad);

   rotMatr._elems[0][0] = cosA;
   rotMatr._elems[0][2] = sinA;
   rotMatr._elems[2][0] = -sinA;
   rotMatr._elems[2][2] = cosA;

   transform(rotMatr, preTransform);
}

void Matr4::rotateZ(Real rotateAngleRad, bool preTransform)
{
   Matr4 rotMatr, resMatr;
   Real sinA = sin(rotateAngleRad), cosA = cos(rotateAngleRad);

   rotMatr._elems[0][0] = cosA;
   rotMatr._elems[0][1] = -sinA;
   rotMatr._elems[1][0] = sinA;
   rotMatr._elems[1][1] = cosA;

   transform(rotMatr, preTransform);
}

void Matr4::transpose(void)
{
   Matr4 copy_this(*this);
   int i, j;

   for (i = 0; i < 4; i++) {
      for (j = 0; j < 4; j++)
         _elems[i][j] = copy_this._elems[j][i];
   }
}

void Matr4::invert(void)
{
   Real det1 = 1.f / det();
   Matr4 invMatr(*this);

   for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
         invMatr._elems[i][j] = det1 * minor(j, i);
      }
   }

   memcpy(_elems, invMatr._elems, sizeof(_elems));

   return;   
}

Real Matr4::det(void) const
{
   Real dt = 0.f;

   dt += _elems[0][0] * (_elems[1][1]*_elems[2][2] - _elems[1][2]*_elems[2][1]) - 
         _elems[0][1] * (_elems[1][0]*_elems[2][2] - _elems[1][2]*_elems[2][0]) + 
         _elems[0][2] * (_elems[1][0]*_elems[2][1] - _elems[1][1]*_elems[2][0]);

   return dt;
}

Real Matr4::minor(int row, int col) const
{
   Matr4 minorMatr;   
   int i, j, l, k;
   
   for (i = 0, l = 0; i < 4; i++) {
      if (i == row) {
         continue;
      }
      for (j = 0, k = 0; j < 4; j++) {
         if (j == col) {
            continue;
         }
         minorMatr._elems[l][k] = _elems[i][j];
         k++;
      }
      l++;
   }
   
   Real coeff = ((row + col) % 2) ? -1.f : 1.f;
   return coeff * minorMatr.det();
}


Vect4 Matr4::getTranslate(void) const
{
   Vect4 transVect;

   transVect[0] = _elems[0][3];                          
   transVect[1] = _elems[1][3];
   transVect[2] = _elems[2][3];

   return transVect;
}

void Matr4::setTranslate(const Vect4& transVect)
{
   _elems[0][3] = transVect[0];
   _elems[1][3] = transVect[1];
   _elems[2][3] = transVect[2];
   _elems[3][3] = 1;
}

Real Matr4::elem(int row, int col) const
{
   return _elems[row][col];
}

Real& Matr4::elem(int row, int col)
{
   return _elems[row][col];
}

Vect4 Matr4::getAxisX(void) const
{
   Vect4 xAxis;

   xAxis[0] = _elems[0][0];
   xAxis[1] = _elems[1][0];
   xAxis[2] = _elems[2][0];

   return xAxis;
}

Vect4 Matr4::getAxisY(void) const
{
   Vect4 yAxis;

   yAxis[0] = _elems[0][1];
   yAxis[1] = _elems[1][1];
   yAxis[2] = _elems[2][1];

   return yAxis;
}

Vect4 Matr4::getAxisZ(void) const
{
   Vect4 zAxis;

   zAxis[0] = _elems[0][2];
   zAxis[1] = _elems[1][2];
   zAxis[2] = _elems[2][2];

   return zAxis;
}

void Matr4::setAxisX(const Vect4& xAxis)
{
   int i;
   for (i = 0; i < 3; i++) {
      _elems[i][0] = xAxis[i];
   }
   return;
}

void Matr4::setAxisY(const Vect4& yAxis)
{
   int i;
   for (i = 0; i < 3; i++) {
      _elems[i][1] = yAxis[i];
   }
   return;
}

void Matr4::setAxisZ(const Vect4& zAxis)
{
   int i;
   for (i = 0; i < 3; i++) {
      _elems[i][2] = zAxis[i];
   }
   return;
}

} // end of namespace ab