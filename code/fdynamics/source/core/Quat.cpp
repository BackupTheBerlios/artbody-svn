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

#include "Quat.h"
#include "Mat3.h"

using namespace mrt;
///////////////////////////////////////////////////////////////////////////////
//  constants
///////////////////////////////////////////////////////////////////////////////

const Quat Quat::ID(Quat::getIDQuat());


///////////////////////////////////////////////////////////////////////////////
//
//  class Quat
//
///////////////////////////////////////////////////////////////////////////////

void Quat::set(Real s, Real x, Real y, Real z) 
{
    s_=s; 
    x_=x; 
    y_=y; 
    z_=z;
}

void Quat::set(Real angle, const Vect3 &axis, int normalizeAxis)
{
  Vect3 axis0;
  Real theta, sine;

  theta = 0.5 * angle;
  sine = sin(theta);
  s_ = cos(theta);

  if (normalizeAxis) {
    axis0.normalize(axis);
    x_ = axis0.x * sine;
    y_ = axis0.y * sine;
    z_ = axis0.z * sine;
  }
  else {
    x_ = axis.x * sine;
    y_ = axis.y * sine;
    z_ = axis.z * sine;
  }

}

  
void Quat::set(const Mat3 &R)
{
  Real qs2, qx2, qy2, qz2;  // squared magniudes of quaternion components
  Real tmp;
  int i;

  // first compute squared magnitudes of quaternion components - at least one
  // will be greater than 0 since quaternion is unit magnitude

  qs2 = 0.25 * (R.xx + R.yy + R.zz + 1);
  qx2 = qs2 - 0.5 * (R.yy + R.zz);
  qy2 = qs2 - 0.5 * (R.zz + R.xx);
  qz2 = qs2 - 0.5 * (R.xx + R.yy);

  
  // find maximum magnitude component
  i = (qs2 > qx2 ) ?
    ((qs2 > qy2) ? ((qs2 > qz2) ? 0 : 3) : ((qy2 > qz2) ? 2 : 3)) :
    ((qx2 > qy2) ? ((qx2 > qz2) ? 1 : 3) : ((qy2 > qz2) ? 2 : 3));

  // compute signed quaternion components using numerically stable method
  switch(i) {
  case 0:
    s_ = sqrt(qs2);
    tmp = 0.25 / s_;
    x_ = (R.zy - R.yz) * tmp;
    y_ = (R.xz - R.zx) * tmp;
    z_ = (R.yx - R.xy) * tmp;
    break;
  case 1:
    x_ = sqrt(qx2);
    tmp = 0.25 / x_;
    s_ = (R.zy - R.yz) * tmp;
    y_ = (R.xy + R.yx) * tmp;
    z_ = (R.xz + R.zx) * tmp;
    break;
  case 2:
    y_ = sqrt(qy2);
    tmp = 0.25 / y_;
    s_ = (R.xz - R.zx) * tmp;
    z_ = (R.yz + R.zy) * tmp;
    x_ = (R.yx + R.xy) * tmp;
    break;
  case 3:
    z_ = sqrt(qz2);
    tmp = 0.25 / z_;
    s_ = (R.yx - R.xy) * tmp;
    x_ = (R.zx + R.xz) * tmp;
    y_ = (R.zy + R.yz) * tmp;
    break;
  }
  // for consistency, force positive scalar component [ (s; v) = (-s; -v) ]
  if (s_ < 0) {
    s_ = -s_;
    x_ = -x_;
    y_ = -y_;
    z_ = -z_;
  }
  // normalize, just to be safe
  tmp = 1.0 / sqrt(s_*s_ + x_*x_ + y_*y_ + z_*z_);
  s_ *= tmp;
  x_ *= tmp;
  y_ *= tmp;
  z_ *= tmp;
}


//ostream& Quat::print(ostream &os) const
//{
//  int oldFlags = os.setf(ios::showpos);
//  os << '(' << s_ << ' ' << x_ << ' ' << y_ << ' ' << z_ << ')';
//  os.flags(oldFlags);
//  return os;
//}


void Quat::mult(const Quat &p, const Quat &q)
{
  s_ = p.s_ * q.s_ - (p.x_ * q.x_ + p.y_ * q.y_ + p.z_ * q.z_);
  x_ = p.s_ * q.x_ +  q.s_ * p.x_ + p.y_ * q.z_ - p.z_ * q.y_;
  y_ = p.s_ * q.y_ +  q.s_ * p.y_ + p.z_ * q.x_ - p.x_ * q.z_;
  z_ = p.s_ * q.z_ +  q.s_ * p.z_ + p.x_ * q.y_ - p.y_ * q.x_;
}


void Quat::premult(const Quat &q)
{
  Real ox, oy, oz;

  ox = x_; oy = y_; oz = z_;

  x_ = q.s_ * ox +  s_ * q.x_ + q.y_ * oz - q.z_ * oy;
  y_ = q.s_ * oy +  s_ * q.y_ + q.z_ * ox - q.x_ * oz;
  z_ = q.s_ * oz +  s_ * q.z_ + q.x_ * oy - q.y_ * ox;
  s_ = q.s_ * s_ - (q.x_ * ox + q.y_ * oy + q.z_ * oz);
}


void Quat::postmult(const Quat &q)
{
  Real ox, oy, oz;

  ox = x_; oy = y_; oz = z_;

  x_ = s_ * q.x_ +  ox * q.s_ + oy * q.z_ - oz * q.y_;
  y_ = s_ * q.y_ +  oy * q.s_ + oz * q.x_ - ox * q.z_;
  z_ = s_ * q.z_ +  oz * q.s_ + ox * q.y_ - oy * q.x_;
  s_ = s_ * q.s_ - (ox * q.x_ + oy * q.y_ + oz * q.z_);
}


// The Quat transformation routines use 19 multiplies and 12 adds
// (counting the multiplications by 2.0).  See Eqn (20) of "A
// Comparison of Transforms and Quaternions in Robotics," Funda and
// Paul, Proceedings of International Conference on Robotics and
// Automation, 1988, p. 886-991.

void Quat::xform(const Vect3 &v, Vect3 &xv) const
{
  Vect3 *u, uv, uuv;


  u = (Vect3 *) &x_;
  uv.cross(*u, v);
  uuv.cross(*u, uv);
  uv.scale(2.0 * s_);
  uuv.scale(2.0);
  xv.add(v, uv);
  xv.add(uuv);
}


void Quat::xform(Vect3 &v) const
{
  Vect3 *u, uv, uuv;

  u = (Vect3 *) &x_;
  uv.cross(*u, v);
  uuv.cross(*u, uv);
  uv.scale(2.0 * s_);
  uuv.scale(2.0);
  v.add(uv);
  v.add(uuv);
}


void Quat::invXform(const Vect3 &v, Vect3 &xv) const
{
  Vect3 *u, uv, uuv;
  
  u = (Vect3 *) &x_;
  uv.cross(*u, v);
  uuv.cross(*u, uv);
  uv.scale(2.0 * -s_);
  uuv.scale(2.0);
  xv.add(v, uv);
  xv.add(uuv);
}


void Quat::invXform(Vect3 &v) const
{
  Vect3 *u, uv, uuv;
  
  u = (Vect3 *) &x_;
  uv.cross(*u, v);
  uuv.cross(*u, uv);
  uv.scale(2.0 * -s_);
  uuv.scale(2.0);
  v.add(uv);
  v.add(uuv);
}

