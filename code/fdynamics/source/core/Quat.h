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

#ifndef QUAT_H
#define QUAT_H

//#ifndef DEPEND_IGNORE
//#include <iostream.h>
//#if INVENTOR
//#include <Inventor/SbLinear.h>
//#endif
//#endif //DEPEND_IGNORE

#include <math.h>

#include "RealType.h"
#include "Vect3.h"

namespace mrt {

class Mat3;
class Se3;

///////////////////////////////////////////////////////////////////////////////
//
//  class Quat
//
///////////////////////////////////////////////////////////////////////////////


class Quat {

  friend class Mat3;
  friend class Se3;

private:

  Real s_, x_, y_, z_;


public:

  // constructors //////////////////////////////////////////////////////////////

  Quat() {}
  Quat(Real s, Real x, Real y, Real z) {set(s, x, y, z);}
  Quat(Real angle, const Vect3 &axis, int normalizeAxis = 1)
    {set(angle, axis, normalizeAxis);}
  Quat(const Mat3 &R) {set(R);}

  // setters / accessors / translators /////////////////////////////////////////

  void set(Real s, Real x, Real y, Real z);
  // set a quaternion to a rotation of 'angle' radians about 'axis'
  // the axis passed is automatically normalized unless normalizeAxis = 0
  void set(Real angle, const Vect3 &axis, int normalizeAxis = 1);
  void set(const Mat3 &R);
  void set(const Quat& another) { s_ = another.s_; x_ = another.x_; y_ = another.y_; z_ = another.z_; }
#if INVENTOR
  inline void set(const SbRotation &R);
  inline void toSbRotation(SbRotation &R) const;
#endif

  Real s() const {return s_;}
  Real x() const {return x_;}
  Real y() const {return y_;}
  Real z() const {return z_;}
  inline Vect3 axis() const;  // normalized axis of rotation
  inline Real angle() const;  // angle of rotation in radians, in range [0, 2pi)

  // input / output ////////////////////////////////////////////////////////////

//  ostream& print(ostream &os) const;

  // operations not returning a Quat ///////////////////////////////////////////

  int operator==(const Quat &other)
    {return s_ == other.s_ && x_ == other.x_ 
         && y_ == other.y_ && z_ == other.z_;}


  // operations returning result via this //////////////////////////////////////

  // The operation result indicated by the comments is always returned
  // in this.  The symbol [!] indicates that this must be distinct
  // from all of the operands.

  inline void normalize(const Quat &q);               // q/|q|
  inline void normalize();                            // this/|this|
  inline void invert(const Quat &q);                  // q^-1
  inline void invert();                               // this^-1
  void mult(const Quat &p, const Quat &q);            // p * q     [!]
  void premult(const Quat &q);                        // q * this  [!]
  void postmult(const Quat &q);                       // this * q  [!]

  // Transforming Vect3s ///////////////////////////////////////////////////////

  void xform(const Vect3 &u, Vect3 &v) const;    // this (v 0) this^-1 => xv
  void xform(Vect3 &v) const;                    // this (v 0) this^-1 => v

  // These are exactly like the above methods, except the inverse
  // transform is used (i.e. the factors this and this^-1 are swapped).
  void invXform(const Vect3 &v, Vect3 &xv) const;
  void invXform(Vect3 &v) const;


  // miscellaneous /////////////////////////////////////////////////////////////

  // Return the quaternion derivatives in 'this', given a current
  // orientation q and body angular velocity w.  Note that what is
  // returned in 'this' is not a unit quaternion, but the derivative
  // of one!
  inline void deriv(const Quat &q, const Vect3 &w);

  // Quat constants ////////////////////////////////////////////////////////////
  static const Quat ID;   // identity quaternion
  static const Quat& getIDQuat(void) { static Quat q(1.0, 0.0, 0.0, 0.0); return q; }
};

///////////////////////////////////////////////////////////////////////////////
//
//  inline function definitions
//
///////////////////////////////////////////////////////////////////////////////



#if INVENTOR
void Quat::set(const SbRotation &rot)
{
  const float *q = rot.getValue();
  s_ = q[3]; x_ = q[0]; y_ = q[1]; z_ = q[2];
}
#endif


#if INVENTOR
void Quat::toSbRotation(SbRotation &rot) const
{
#if OPCOUNTS
  rot.setValue(x_.toDouble(), y_.toDouble(), z_.toDouble(), s_.toDouble());
#else
  rot.setValue(x_, y_, z_, s_);
#endif
}
#endif


Vect3 Quat::axis() const
{
  Vect3 v(x_, y_, z_);
  if (v.norm() == 0.0) v = Vect3::I;  // axis is arbitrary here
  else v.normalize();
  return v;
}


Real Quat::angle() const
{
#if OPCOUNTS
  return 2 * acos(s_.toDouble());
#else
  return 2 * acos(s_);
#endif
}


void Quat::normalize(const Quat &q)
{
  Real scale;

  scale = 1.0 / sqrt(q.s_*q.s_ + q.x_*q.x_ + q.y_*q.y_ + q.z_*q.z_);
  s_ = scale * q.s_;
  x_ = scale * q.x_;
  y_ = scale * q.y_;
  z_ = scale * q.z_;
}


void Quat::normalize()
{
  Real scale;

  scale = 1.0 / sqrt(s_*s_ + x_*x_ + y_*y_ + z_*z_);
  s_ *= scale;
  x_ *= scale;
  y_ *= scale;
  z_ *= scale;
}


void Quat::invert(const Quat &q)
{
  s_ = -q.s_;
  x_ =  q.x_;
  y_ =  q.y_;
  z_ =  q.z_;
}


void Quat::invert()
{
  s_ = -s_;
}


void Quat::deriv(const Quat &q, const Vect3 &w)
{
  s_ = 0.5 * (-q.x_ * w.x - q.y_ * w.y - q.z_ * w.z);
  x_ = 0.5 * ( q.s_ * w.x - q.z_ * w.y + q.y_ * w.z);
  y_ = 0.5 * ( q.z_ * w.x + q.s_ * w.y - q.x_ * w.z);
  z_ = 0.5 * (-q.y_ * w.x + q.x_ * w.y + q.s_ * w.z);
}

///////////////////////////////////////////////////////////////////////////////
//
//  stream operators
//
///////////////////////////////////////////////////////////////////////////////

//inline ostream &operator<<(ostream &os, const Quat &q)  {return q.print(os);}

} // namespace mrt


#endif // QUAT_H

