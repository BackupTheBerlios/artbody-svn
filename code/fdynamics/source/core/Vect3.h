///////////////////////////////////////////////////////////////////////////////
//
//  Copyright 1998 Mitsubishi Electric Information Technology Center
//  America (MEITCA).  All Rights Reserved.
//
//  Permission to use, copy, modify and distribute this software and
//  its documentation for educational, research and non-profit
//  purposes, without fee, and without a written agreement is hereby
//  granted, provided that the above copyright notice and the
//  following three paragraphs appear in all copies.
//
//  Permission to incorporate this software into commercial products
//  may be obtained from MERL - A Mitsubishi Electric Research Lab, 201
//  Broadway, Cambridge, MA 02139.
//
//  IN NO EVENT SHALL MEITCA BE LIABLE TO ANY PARTY FOR DIRECT,
//  INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
//  LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
//  DOCUMENTATION, EVEN IF MEITCA HAS BEEN ADVISED OF THE POSSIBILITY
//  OF SUCH DAMAGES.
//
//  MEITCA SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS ON
//  AN "AS IS" BASIS, AND MEITCA HAS NO OBLIGATIONS TO PROVIDE
//  MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
//  Author:
//    Brian Mirtich
//    mirtich@merl.com
//    617.621.7573
//    www.merl.com/people/mirtich
//
//  Library and API:
//    James Kuffner, Jr.
//    kuffner@stanford.edu
//    650.725.8812
//    http://robotics.stanford.edu/~kuffner
//
///////////////////////////////////////////////////////////////////////////////
#ifndef VECT3_H
#define VECT3_H

//#ifndef DEPEND_IGNORE
//#include <iostream.h>
//#if INVENTOR
//#include <Inventor/SbLinear.h>
//#else
#include <math.h>
//#endif
//#endif //DEPEND_IGNORE

#include "RealType.h"

namespace mrt {

class Mat3;
class MatX;
class Quat;
class Se3;


///////////////////////////////////////////////////////////////////////////////
//
//  class Vect3
//
///////////////////////////////////////////////////////////////////////////////


class Vect3 {

friend class Mat3;
friend class MatX;
friend class Quat;
friend class Se3;

public:

  Real x, y, z;

  // constructors //////////////////////////////////////////////////////////////

  Vect3() {}
  Vect3(Real x_, Real y_, Real z_) {set(x_, y_, z_);}

  // setters / accessors / translators /////////////////////////////////////////

  void set(Real x_, Real y_, Real z_) {x = x_; y = y_; z = z_;}

  //Real &coord(int i) {return *(&x + i);} // index-based access:  0=x, 1=y, 2=z.

  // index-based access:  0=x, 1=y, 2=z.
  const Real &operator[](int i) const {return *(&x + i);} 
        Real &operator[](int i)       {return *(&x + i);} 


#if INVENTOR
  inline void set(const SbVec3f &v);
  inline void toSbVec3f(SbVec3f &v) const;
#endif

  // input / output ////////////////////////////////////////////////////////////

//  ostream &print(ostream &os) const;
//  // Read vector from stream If the next string read is the single
//  // character i, j, or k, the appropriate unit vector is returned.
//  // Plus and minus signs may be optionally placed in front of these
//  // codes, e.g. +i or -k
//  istream &read(istream &is);

  // operations not returning a Vect3 //////////////////////////////////////////

  //inline int operator==(const Vect3 &u, const Vect3 &v);
  inline int operator==(const Vect3 &other) const;
  inline Real dot(const Vect3 &other) const;
  inline Real norm()  const;
  inline Real norm2() const;  // norm^2
  inline Real distance (const Vect3 &other) const;
  inline Real distance2(const Vect3 &other) const;  // distance^2
  inline Real min() const;
  inline Real max() const;
  inline Real minAbs() const;
  inline Real maxAbs() const;
  inline void swap(Vect3 &other);
  // for symmetric invocations:
  static Real dot      (const Vect3 &u, const Vect3 &v) {return u.dot      (v);}
  static Real distance (const Vect3 &u, const Vect3 &v) {return u.distance (v);}
  static Real distance2(const Vect3 &u, const Vect3 &v) {return u.distance2(v);}
  static void swap     (      Vect3 &u,       Vect3 &v) {       u.swap     (v);}
  
  // operations returning result via this //////////////////////////////////////

  // The operation result indicated by the comments is always returned
  // in this.  The symbol [!] indicates that this must be distinct
  // from all of the operands.  The three-argument crossAdd() is
  // slightly different: this must be distinct from u and v, but not
  // necessarily from w.

  inline void normalize(const Vect3 &v);            // v/|v|
  inline void normalize();                          // this/|this|
  inline void negate(const Vect3 &v);               // -v
  inline void negate();                             // -this
  inline void add(const Vect3 &u, const Vect3 &v);  // u + v
  inline void add(const Vect3 &v);                  // this + v
  inline void sub(const Vect3 &u, const Vect3 &v);  // u - v
  inline void sub(const Vect3 &v);                  // this - v
  inline void mult(const Vect3 &u, const Vect3 &v); // u * v (component-wise) 
  inline void mult(const Vect3 &v);                 // this * v (component-wise)
  inline void scale(const Vect3 &v, Real s);        // s * v
  inline void scale(Real s);                        // s * this
  inline void cross(const Vect3 &u, const Vect3 &v);// u x v  [!]
  inline void precross(const Vect3 &v);             // v x this  [!]
  inline void postcross(const Vect3 &v);            // this x v  [!]
  inline void crossAdd(const Vect3 &u, const Vect3 &v, const Vect3 &w);
                                                    // u x v + w [!]
  inline void crossAdd(const Vect3 &u, const Vect3 &v);
                                                    //  u x v + this [!]
  inline void displace(const Vect3 &v, const Vect3 &u, Real lambda); 
                                                    // v + lambda * u
  inline void displace(const Vect3 &u, Real lambda);// this + lambda * u
  inline void interpolate(const Vect3 &u, const Vect3 &v, Real lambda);  
                                                    // (1-lambda)*u + lambda*v


  // Vect3 constants ///////////////////////////////////////////////////////////

  static const Vect3 ZERO;
  static const Vect3 I;     // unit vector along +x axis
  static const Vect3 J;     // unit vector along +y axis
  static const Vect3 K;     // unit vector along +z axis
  static const Vect3 I_;    // unit vector along -x axis
  static const Vect3 J_;    // unit vector along -y axis
  static const Vect3 K_;    // unit vector along -z axis

};

///////////////////////////////////////////////////////////////////////////////
//
//  inline function definitions
//
///////////////////////////////////////////////////////////////////////////////


#if INVENTOR
void Vect3::set(const SbVec3f &vec)
{
  const float *v = vec.getValue();
  x = v[0]; y = v[1]; z = v[2];
}
#endif


#if INVENTOR
void Vect3::toSbVec3f(SbVec3f &vec) const
{
#if OPCOUNTS
  vec.setValue(x.toDouble(), y.toDouble(), z.toDouble());
#else
  vec.setValue(x, y, z);
#endif
}
#endif


int Vect3::operator==(const Vect3 &other) const
{
  return x == other.x && y == other.y && z == other.z;
}


Real Vect3::dot(const Vect3 &other) const
{
  return x * other.x + y * other.y + z * other.z;
}


Real Vect3::norm() const
{
  return sqrt(x * x + y * y + z * z);
}


Real Vect3::norm2() const
{
  return (x * x + y * y + z * z);
}


Real Vect3::distance(const Vect3 &other) const
{
  Vect3 w;

  w.sub(other, *this);
  return w.norm();
}


Real Vect3::distance2(const Vect3 &other) const
{
  Vect3 w;

  w.sub(other, *this);
  return w.norm2();
}


Real Vect3::min() const
{
  return (x <= y) ? ((x <= z) ? x : z) : ((y <= z) ? y : z);
}


Real Vect3::max() const
{
  return (x >= y) ? ((x >= z) ? x : z) : ((y >= z) ? y : z);
}


Real Vect3::minAbs() const
{
  Real ax, ay, az;
  
  ax = fabs(x);
  ay = fabs(y);
  az = fabs(z);
  return (ax <= ay) ? ((ax <= az) ? ax : az) : ((ay <= az) ? ay : az);
}


Real Vect3::maxAbs() const
{
  Real ax, ay, az;
  
  ax = fabs(x);
  ay = fabs(y);
  az = fabs(z);
  return (ax >= ay) ? ((ax >= az) ? ax : az) : ((ay >= az) ? ay : az);
}


void Vect3::swap(Vect3 &other)
{
  Vect3 tmp;

  tmp = *this;
  *this = other;
  other = tmp;
}


void Vect3::normalize(const Vect3 &v)
{
  Real s;

  s = 1.0 / sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  x = s * v.x;
  y = s * v.y;
  z = s * v.z;
}


void Vect3::normalize()
{
  Real s;

  s = 1.0 / sqrt(x * x + y * y + z * z);
  x *= s;
  y *= s;
  z *= s;
}


void Vect3::negate(const Vect3 &v)
{
  x = - v.x;
  y = - v.y;
  z = - v.z;
}


void Vect3::negate()
{
  x = - x;
  y = - y;
  z = - z;
}


void Vect3::add(const Vect3 &u, const Vect3 &v)
{
  x = u.x + v.x;
  y = u.y + v.y;
  z = u.z + v.z;
}


void Vect3::add(const Vect3 &v)
{
  x += v.x;
  y += v.y;
  z += v.z;
}


void Vect3::sub(const Vect3 &u, const Vect3 &v)
{
  x = u.x - v.x;
  y = u.y - v.y;
  z = u.z - v.z;
}


void Vect3::sub(const Vect3 &v)
{
  x -= v.x;
  y -= v.y;
  z -= v.z;
}


void Vect3::mult(const Vect3 &u, const Vect3 &v)
{
  x = u.x * v.x;
  y = u.y * v.y;
  z = u.z * v.z;
}


void Vect3::mult(const Vect3 &v)
{
  x *= v.x;
  y *= v.y;
  z *= v.z;
}


void Vect3::scale(const Vect3 &v, Real s)
{
  x = s * v.x;
  y = s * v.y;
  z = s * v.z;
}


void Vect3::scale(Real s)
{
  x *= s;
  y *= s;
  z *= s;
}



void Vect3::cross(const Vect3 &u, const Vect3 &v)
{
  x = u.y * v.z - u.z * v.y;
  y = u.z * v.x - u.x * v.z;
  z = u.x * v.y - u.y * v.x;
}


void Vect3::precross(const Vect3 &v)
{
  Real ox, oy;

  ox = x;
  oy = y;
  x = v.y * z - v.z * oy;
  y = v.z * ox - v.x * z;
  z = v.x * oy - v.y * ox;
}


void Vect3::postcross(const Vect3 &v)
{
  Real ox, oy;

  ox = x;
  oy = y;
  x = oy * v.z - z * v.y;
  y = z * v.x - ox * v.z;
  z = ox * v.y - oy * v.x;
}


void Vect3::crossAdd(const Vect3 &u, const Vect3 &v, const Vect3 &w)
{
  x = u.y * v.z - u.z * v.y + w.x;
  y = u.z * v.x - u.x * v.z + w.y;
  z = u.x * v.y - u.y * v.x + w.z;
}


void Vect3::crossAdd(const Vect3 &u, const Vect3 &v)
{
  x += u.y * v.z - u.z * v.y;
  y += u.z * v.x - u.x * v.z;
  z += u.x * v.y - u.y * v.x;
}


void Vect3::displace(const Vect3 &v, const Vect3 &u, Real lambda)
{
  x = v.x + lambda * u.x;
  y = v.y + lambda * u.y;
  z = v.z + lambda * u.z;
}


void Vect3::displace(const Vect3 &u, Real lambda)
{
  x += lambda * u.x;
  y += lambda * u.y;
  z += lambda * u.z;
}


void Vect3::interpolate(const Vect3 &u, const Vect3 &v, Real lambda)
{
  Real lambda2 = 1.0 - lambda;

  x = lambda2 * u.x + lambda * v.x;
  y = lambda2 * u.y + lambda * v.y;
  z = lambda2 * u.z + lambda * v.z;
}


///////////////////////////////////////////////////////////////////////////////
//
//  stream operators
//
///////////////////////////////////////////////////////////////////////////////

//inline ostream &operator<<(ostream &os, const Vect3 &v) {return v.print(os);}
//inline istream &operator>>(istream &os, Vect3 &v)       {return v.read(os);}

} // namespace mrt

#endif // VECT3_H
