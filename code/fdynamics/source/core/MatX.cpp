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

#include "MatX.h"
#include "Se3.h"

using namespace mrt;


///////////////////////////////////////////////////////////////////////////////
//  constants
///////////////////////////////////////////////////////////////////////////////

const MatX MatX::ID   (Mat3::ID, Vect3::ZERO);


///////////////////////////////////////////////////////////////////////////////
//
//  class MatX
//
///////////////////////////////////////////////////////////////////////////////


void MatX::mult(const MatX &M, const MatX &N)
{
  // multiply rotation matrices
  R.xx = M.R.xx * N.R.xx + M.R.xy * N.R.yx + M.R.xz * N.R.zx;
  R.xy = M.R.xx * N.R.xy + M.R.xy * N.R.yy + M.R.xz * N.R.zy;
  R.xz = M.R.xx * N.R.xz + M.R.xy * N.R.yz + M.R.xz * N.R.zz;
  R.yx = M.R.yx * N.R.xx + M.R.yy * N.R.yx + M.R.yz * N.R.zx;
  R.yy = M.R.yx * N.R.xy + M.R.yy * N.R.yy + M.R.yz * N.R.zy;
  R.yz = M.R.yx * N.R.xz + M.R.yy * N.R.yz + M.R.yz * N.R.zz;
  R.zx = M.R.zx * N.R.xx + M.R.zy * N.R.yx + M.R.zz * N.R.zx;
  R.zy = M.R.zx * N.R.xy + M.R.zy * N.R.yy + M.R.zz * N.R.zy;
  R.zz = M.R.zx * N.R.xz + M.R.zy * N.R.yz + M.R.zz * N.R.zz;

  // d = M.R * N.d + M.d
  d.x = M.R.xx * N.d.x + M.R.xy * N.d.y + M.R.xz * N.d.z + M.d.x;
  d.y = M.R.yx * N.d.x + M.R.yy * N.d.y + M.R.yz * N.d.z + M.d.y;
  d.z = M.R.zx * N.d.x + M.R.zy * N.d.y + M.R.zz * N.d.z + M.d.z;
}


void MatX::premult(const MatX &M)
{
  Real oxy, oyz, ozx, oyx, ozy, oxz, odx, ody;


  // multiply rotation matrices
  oxy = R.xy; oyx = R.yx; oyz = R.yz; ozy = R.zy; ozx = R.zx; oxz = R.xz;

  R.xy = M.R.xx * oxy  + M.R.xy * R.yy + M.R.xz * ozy;
  R.xz = M.R.xx * oxz  + M.R.xy * oyz  + M.R.xz * R.zz;
  R.yx = M.R.yx * R.xx + M.R.yy * oyx  + M.R.yz * ozx;
  R.yz = M.R.yx * oxz  + M.R.yy * oyz  + M.R.yz * R.zz;
  R.zx = M.R.zx * R.xx + M.R.zy * oyx  + M.R.zz * ozx;
  R.zy = M.R.zx * oxy  + M.R.zy * R.yy + M.R.zz * ozy;

  R.xx = M.R.xx * R.xx + M.R.xy * oyx  + M.R.xz * ozx;
  R.yy = M.R.yx * oxy  + M.R.yy * R.yy + M.R.yz * ozy;
  R.zz = M.R.zx * oxz  + M.R.zy * oyz  + M.R.zz * R.zz;

  // d = M.R * d + M.d
  odx = d.x; ody = d.y;
  d.x = M.R.xx * odx + M.R.xy * ody + M.R.xz * d.z + M.d.x;
  d.y = M.R.yx * odx + M.R.yy * ody + M.R.yz * d.z + M.d.y;
  d.z = M.R.zx * odx + M.R.zy * ody + M.R.zz * d.z + M.d.z;
}


void MatX::postmult(const MatX &M)
{
  Real oxy, oyz, ozx, oyx, ozy, oxz;
  Vect3 v;

  // d = R * M.d + d
  d.x += R.xx * M.d.x + R.xy * M.d.y + R.xz * M.d.z;
  d.y += R.yx * M.d.x + R.yy * M.d.y + R.yz * M.d.z;
  d.z += R.zx * M.d.x + R.zy * M.d.y + R.zz * M.d.z;

  // multiply rotation matrices
  oxy = R.xy; oyx = R.yx; oyz = R.yz; ozy = R.zy; ozx = R.zx; oxz = R.xz;
  R.xy = R.xx * M.R.xy + oxy  * M.R.yy + oxz  * M.R.zy;
  R.xz = R.xx * M.R.xz + oxy  * M.R.yz + oxz  * M.R.zz;
  R.yx = oyx  * M.R.xx + R.yy * M.R.yx + oyz  * M.R.zx;
  R.yz = oyx  * M.R.xz + R.yy * M.R.yz + oyz  * M.R.zz;
  R.zx = ozx  * M.R.xx + ozy  * M.R.yx + R.zz * M.R.zx;
  R.zy = ozx  * M.R.xy + ozy  * M.R.yy + R.zz * M.R.zy;

  R.xx = R.xx * M.R.xx + oxy  * M.R.yx + oxz  * M.R.zx;
  R.yy = oyx  * M.R.xy + R.yy * M.R.yy + oyz  * M.R.zy;
  R.zz = ozx  * M.R.xz + ozy  * M.R.yz + R.zz * M.R.zz;
}


void MatX::invert(const MatX &M)
{
  // invert the rotation part by transposing it
  R.xx = M.R.xx;
  R.xy = M.R.yx;
  R.xz = M.R.zx;
  R.yx = M.R.xy;
  R.yy = M.R.yy;
  R.yz = M.R.zy;
  R.zx = M.R.xz;
  R.zy = M.R.yz;
  R.zz = M.R.zz;

  // new displacement vector given by:  d' = -(R^-1) * d 
  d.x = - (R.xx * M.d.x + R.xy * M.d.y + R.xz * M.d.z);
  d.y = - (R.yx * M.d.x + R.yy * M.d.y + R.yz * M.d.z);
  d.z = - (R.zx * M.d.x + R.zy * M.d.y + R.zz * M.d.z);
}



void MatX::invert()
{
  Real tmp, odx, ody;

  // invert the rotation part by transposing it
  tmp  = R.xy;
  R.xy = R.yx;
  R.yx = tmp;

  tmp  = R.yz;
  R.yz = R.zy;
  R.zy = tmp;

  tmp  = R.zx;
  R.zx = R.xz;
  R.xz = tmp;

  // new displacement vector given by:  d' = -(R^T) * d 
  odx = d.x; ody = d.y;
  d.x = - (R.xx * odx + R.xy * ody + R.xz * d.z);
  d.y = - (R.yx * odx + R.yy * ody + R.yz * d.z);
  d.z = - (R.zx * odx + R.zy * ody + R.zz * d.z);
}



void MatX::set(const Se3 &T)
{
  R.set(T.q); 
  d = T.d;
}


//istream& MatX::read(istream &is)
//{
//  Se3 T;
//  is >> T;
//  set(T);
//  return is;
//}

