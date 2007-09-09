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

#include "Mat3.h"

using namespace mrt;

///////////////////////////////////////////////////////////////////////////////
//  constants
///////////////////////////////////////////////////////////////////////////////

const Mat3 Mat3::ZERO (Vect3::ZERO, Vect3::ZERO);
const Mat3 Mat3::ID   (Vect3(1, 1, 1), Vect3::ZERO);


///////////////////////////////////////////////////////////////////////////////
//
//  class Mat3
//
///////////////////////////////////////////////////////////////////////////////


void Mat3::set(const Quat &q)
{
  xx = 2.0 * (q.s_ * q.s_ + q.x_ * q.x_ - 0.5);
  yy = 2.0 * (q.s_ * q.s_ + q.y_ * q.y_ - 0.5);
  zz = 2.0 * (q.s_ * q.s_ + q.z_ * q.z_ - 0.5);

  xy = 2.0 * (q.y_ * q.x_ - q.z_ * q.s_);
  yx = 2.0 * (q.x_ * q.y_ + q.z_ * q.s_);


  yz = 2.0 * (q.z_ * q.y_ - q.x_ * q.s_);
  zy = 2.0 * (q.y_ * q.z_ + q.x_ * q.s_);

  zx = 2.0 * (q.x_ * q.z_ - q.y_ * q.s_);
  xz = 2.0 * (q.z_ * q.x_ + q.y_ * q.s_);
}


//ostream& Mat3::print(ostream &os) const
//{
//  int oldFlags = os.setf(ios::showpos);
//  os << '[' << xx << ' ' << xy << ' ' << xz << ']' << endl;
//  os << '[' << yx << ' ' << yy << ' ' << yz << ']' << endl;
//  os << '[' << zx << ' ' << zy << ' ' << zz << ']' << endl;
//  os.flags(oldFlags);
//  return os;
//}


int Mat3::invert(const Mat3 &M)
{
  Real D, oneOverDet;

  if (fabs(D = M.det()) < 1.0e-12) return 1; // not invertible
  oneOverDet = 1 / D;

  xx = (M.yy * M.zz - M.yz * M.zy) * oneOverDet;
  xy = (M.xz * M.zy - M.xy * M.zz) * oneOverDet;
  xz = (M.xy * M.yz - M.xz * M.yy) * oneOverDet;
  yx = (M.yz * M.zx - M.yx * M.zz) * oneOverDet;
  yy = (M.xx * M.zz - M.xz * M.zx) * oneOverDet;
  yz = (M.xz * M.yx - M.xx * M.yz) * oneOverDet;
  zx = (M.yx * M.zy - M.yy * M.zx) * oneOverDet;
  zy = (M.xy * M.zx - M.xx * M.zy) * oneOverDet;
  zz = (M.xx * M.yy - M.xy * M.yx) * oneOverDet;
  return 0;
}


int Mat3::invert()
{
  Real D, oneOverDet;
  Real oxx, oyy, oxy, oyz, ozx, oyx, ozy, oxz;

  if (fabs(D = det()) < 1.0e-12) return 1; // not invertible
  oneOverDet = 1 / D;

  oxx = xx; oyy = yy;
  oxy = xy; oyx = yx;
  oyz = yz; ozy = zy;
  ozx = zx; oxz = xz;

  xy = (oxz * ozy - zz * oxy) * oneOverDet;
  yz = (oxz * oyx - xx * oyz) * oneOverDet;
  zx = (oyx * ozy - yy * ozx) * oneOverDet;
  yx = (oyz * ozx - oyx * zz) * oneOverDet;
  zy = (oxy * ozx - ozy * xx) * oneOverDet;
  xz = (oxy * oyz - oxz * yy) * oneOverDet;
  xx = ( yy *  zz - oyz * ozy) * oneOverDet;
  yy = (oxx *  zz - oxz * ozx) * oneOverDet;
  zz = (oxx * oyy - oxy * oyx) * oneOverDet;
  return 0;
}


void Mat3::mult(const Mat3 &M, const Mat3 &N)
{
  xx = M.xx * N.xx + M.xy * N.yx + M.xz * N.zx;
  xy = M.xx * N.xy + M.xy * N.yy + M.xz * N.zy;
  xz = M.xx * N.xz + M.xy * N.yz + M.xz * N.zz;
  yx = M.yx * N.xx + M.yy * N.yx + M.yz * N.zx;
  yy = M.yx * N.xy + M.yy * N.yy + M.yz * N.zy;
  yz = M.yx * N.xz + M.yy * N.yz + M.yz * N.zz;
  zx = M.zx * N.xx + M.zy * N.yx + M.zz * N.zx;
  zy = M.zx * N.xy + M.zy * N.yy + M.zz * N.zy;
  zz = M.zx * N.xz + M.zy * N.yz + M.zz * N.zz;
}


void Mat3::premult(const Mat3 &M)
{
  Real oxy, oyz, ozx, oyx, ozy, oxz;

  oxy = xy; oyx = yx; oyz = yz; ozy = zy; ozx = zx; oxz = xz;

  xy = M.xx * oxy + M.xy * yy  + M.xz * ozy;
  xz = M.xx * oxz + M.xy * oyz + M.xz * zz;
  yx = M.yx * xx  + M.yy * oyx + M.yz * ozx;
  yz = M.yx * oxz + M.yy * oyz + M.yz * zz;
  zx = M.zx * xx  + M.zy * oyx + M.zz * ozx;
  zy = M.zx * oxy + M.zy * yy  + M.zz * ozy;

  xx = M.xx * xx  + M.xy * oyx + M.xz * ozx;
  yy = M.yx * oxy + M.yy * yy  + M.yz * ozy;
  zz = M.zx * oxz + M.zy * oyz + M.zz * zz;
}


void Mat3::postmult(const Mat3 &M)
{
  Real oxy, oyz, ozx, oyx, ozy, oxz;

  oxy = xy; oyx = yx; oyz = yz; ozy = zy; ozx = zx; oxz = xz;

  xy = xx *  M.xy + oxy * M.yy + oxz * M.zy;
  xz = xx *  M.xz + oxy * M.yz + oxz * M.zz;
  yx = oyx * M.xx + yy  * M.yx + oyz * M.zx;
  yz = oyx * M.xz + yy  * M.yz + oyz * M.zz;
  zx = ozx * M.xx + ozy * M.yx + zz  * M.zx;
  zy = ozx * M.xy + ozy * M.yy + zz  * M.zy;

  xx = xx  * M.xx + oxy * M.yx + oxz * M.zx;
  yy = oyx * M.xy + yy  * M.yy + oyz * M.zy;
  zz = ozx * M.xz + ozy * M.yz + zz  * M.zz;
}

