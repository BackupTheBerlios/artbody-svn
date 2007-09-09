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

#include <math.h>
#include <string.h>

#include "Mat3.h"
#include "Spatial.h"

using namespace mrt;

///////////////////////////////////////////////////////////////////////////////
//  constants
///////////////////////////////////////////////////////////////////////////////


const SpatialVect SpatialVect::ZERO = SpatialVect(Vect3::ZERO, Vect3::ZERO);

const SpatialMat SpatialMat::ZERO
  = SpatialMat(Mat3::ZERO, Mat3::ZERO, Mat3::ZERO, Mat3::ZERO);


///////////////////////////////////////////////////////////////////////////////
//  class SpatialVect
///////////////////////////////////////////////////////////////////////////////


//ostream& operator<<(ostream &os, const SpatialVect &v)
//{
//  int oldFlags = os.setf(ios::showpos);
//  os << '(' << v._top.x << ' ' << v._top.y << ' ' << v._top.z << ' ; '
//            << v._bot.x << ' ' << v._bot.y << ' ' << v._bot.z << ')';
//  os.flags(oldFlags);
//  return os;
//}


int mrt::operator==(const SpatialVect &u, const SpatialVect &v)
{
  return u._top == v._top && u._bot == v._bot;
}


Real SpatialVect::dot(const SpatialVect &other) const 
{
  return _top.dot(other._bot) + _bot.dot(other._top);
}


// negate vector:  -v => this
void SpatialVect::negate(const SpatialVect &v)
{
  _top.negate(v._top);
  _bot.negate(v._bot);
}


// negate vector:  -this => this
void SpatialVect::negate()
{
  _top.negate();
  _bot.negate();
}


// add vectors:  u + v => this
void SpatialVect::add(const SpatialVect &u, const SpatialVect &v)
{
  _top.add(u._top, v._top);
  _bot.add(u._bot, v._bot);
}


// add vectors:  this + v => this
void SpatialVect::add(const SpatialVect &v)
{
  _top.add(v._top);
  _bot.add(v._bot);
}


// sub vectors:  u - v => this
void SpatialVect::sub(const SpatialVect &u, const SpatialVect &v)
{
  _top.sub(u._top, v._top);
  _bot.sub(u._bot, v._bot);
}


// sub vectors:  this - v => this
void SpatialVect::sub(const SpatialVect &v)
{
  _top.sub(v._top);
  _bot.sub(v._bot);
}


// scale vector:  s * v => this
void SpatialVect::scale(const SpatialVect &v, Real s)
{
  _top.scale(v._top, s);
  _bot.scale(v._bot, s);
}


// scale vector:  s * this => this
void SpatialVect::scale(Real s)
{
  _top.scale(s);
  _bot.scale(s);
}


///////////////////////////////////////////////////////////////////////////////
//  class SpatialMat
///////////////////////////////////////////////////////////////////////////////


//ostream& operator<<(ostream &os, const SpatialMat &M)
//{
//  int oldFlags = os.setf(ios::showpos);
//  os << '[' << M._A.xrow().x << ' ' << M._A.xrow().y << ' ' << M._A.xrow().z
//     << ' ' << M._B.xrow().x << ' ' << M._B.xrow().y << ' ' << M._B.xrow().z 
//     << ']' << endl;
//  os << '[' << M._A.yrow().x << ' ' << M._A.yrow().y << ' ' << M._A.yrow().z
//     << ' ' << M._B.yrow().x << ' ' << M._B.yrow().y << ' ' << M._B.yrow().z 
//     << ']' << endl;
//  os << '[' << M._A.zrow().x << ' ' << M._A.zrow().y << ' ' << M._A.zrow().z
//     << ' ' << M._B.zrow().x << ' ' << M._B.zrow().y << ' ' << M._B.zrow().z 
//     << ']' << endl;
//  os << '[' << M._C.xrow().x << ' ' << M._C.xrow().y << ' ' << M._C.xrow().z
//     << ' ' << M._D.xrow().x << ' ' << M._D.xrow().y << ' ' << M._D.xrow().z 
//     << ']' << endl;
//  os << '[' << M._C.yrow().x << ' ' << M._C.yrow().y << ' ' << M._C.yrow().z
//     << ' ' << M._D.yrow().x << ' ' << M._D.yrow().y << ' ' << M._D.yrow().z 
//     << ']' << endl;
//  os << '[' << M._C.zrow().x << ' ' << M._C.zrow().y << ' ' << M._C.zrow().z
//     << ' ' << M._D.zrow().x << ' ' << M._D.zrow().y << ' ' << M._D.zrow().z 
//     << ']' << endl;
//  os.flags(oldFlags);
//  return os;
//}


// invert spatial articulated inertia matrix

// The inversion method is based on partitioning (see Numerical
// Recipes, p. 77), but differs in two respects.  First, we avoid
// inverting the op-left and bottom-right 3 x 3 submatrices since
// these are often singular.  Second, we exploit the symmetries in the
// 4 3x3 components of the inertia matrix I, namely A^T = D, B^T = B,
// and C^T = C.  See my notes for details.  N.B.  This is not a
// general 6 x 6 matrix inversion routine!

// This routine is only needed for multibodies which are floating,
// i.e. not anchored to an inertial frame via joint.

void SpatialMat::inertiaInvert(const SpatialMat &I)
{
  Mat3 Binv;
  Mat3 M, N;  /* temp storage */

  
  Binv.invert(I._B);
  Binv.negate();
  M.mult(I._D, Binv);
  N.mult(M, I._A);
  N.add(I._C);
  _B.invert(N);

  M.mult(_B, I._D);
  _A.mult(M, Binv);
  
  _D.xpose(_A);

  M.mult(I._A, _A);
  M.xrow().x -= 1.0; 
  M.yrow().y -= 1.0; 
  M.zrow().z -= 1.0;
  _C.mult(Binv, M);
  
  //Mat3 detI;

  //detI.mult(I._A, I._D);
  //M.mult(I._B, I._C);
  //detI.sub(M);
  //detI.invert();

  //_A.mult(detI, I._D);
  //_B.mult(detI, I._B);
  //_B.negate();
  //_C.mult(detI, I._C);
  //_C.negate();
  //_D.mult(detI, I._A);

  return;
}


// add matrices:  M + N => this
void SpatialMat::add(const SpatialMat &M, const SpatialMat &N)
{
  _A.add(M._A, N._A);
  _B.add(M._B, N._B);
  _C.add(M._C, N._C);
  _D.add(M._D, N._D);
}


// add matrices:  this + M => this
void SpatialMat::add(const SpatialMat &M)
{
  _A.add(M._A);
  _B.add(M._B);
  _C.add(M._C);
  _D.add(M._D);
}


// sub matrices:  M - N => this
void SpatialMat::sub(const SpatialMat &M, const SpatialMat &N)
{
  _A.sub(M._A, N._A);
  _B.sub(M._B, N._B);
  _C.sub(M._C, N._C);
  _D.sub(M._D, N._D);
}


// sub matrices:  this - M => this
void SpatialMat::sub(const SpatialMat &M)
{
  _A.sub(M._A);
  _B.sub(M._B);
  _C.sub(M._C);
  _D.sub(M._D);
}


// scale matrix:  s * M => this
void SpatialMat::scale(const SpatialMat &M, Real s)
{
  _A.scale(M._A, s);
  _B.scale(M._B, s);
  _C.scale(M._C, s);
  _D.scale(M._D, s);
}


// scale matrix:  s * this => this
void SpatialMat::scale(Real s)
{
  _A.scale(s);
  _B.scale(s);
  _C.scale(s);
  _D.scale(s);
}


// xform a spatial vector by a spatial matrix
void SpatialMat::xform(const SpatialVect &u, SpatialVect &v) const
{
  Vect3 tmp;

  _A.xform(u.top(), v.top());
  _B.xform(u.bot(), tmp);
  v.top().add(tmp);

  _C.xform(u.top(), v.bot());
  _D.xform(u.bot(), tmp);
  v.bot().add(tmp);
}



