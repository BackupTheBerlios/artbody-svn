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

#ifndef SPATIAL_H
#define SPATIAL_H

#include <math.h>

#include "Mat3.h"

#define DEG_TO_RAD (3.14158 / 180.0)

namespace mrt {

///////////////////////////////////////////////////////////////////////////////
//
//  class SpatialVect
//
//  SpatialVects are 6-dimensional vectors used for multibody dynamics
//  algorithms.
//
///////////////////////////////////////////////////////////////////////////////


class SpatialVect {

private:
  Vect3 _top, _bot;

public:

  // constructors

  SpatialVect() {}
  SpatialVect(const Vect3 &top, const Vect3 &bot) {_top = top; _bot = bot;}

  void set(const Vect3 &top, const Vect3 &bot) {_top = top; _bot = bot;}

  // accessors
  const Vect3 &top() const { return _top; }
        Vect3 &top()       { return _top; }
  const Vect3 &bot() const { return _bot; }
        Vect3 &bot()       { return _bot; }

  
  // input / output

//  friend ostream& operator<<(ostream &os, const SpatialVect &v);

  // operations not returning a SpatialVect

  friend int operator==(const SpatialVect &u, const SpatialVect &v);
  Real dot(const SpatialVect &other) const;
  //  void swap(SpatialVect &other);
  
  // operations returning a SpatialVect via this

  void negate(const SpatialVect &v);
  void negate();
  void add(const SpatialVect &u, const SpatialVect &v);
  void add(const SpatialVect &v);
  void sub(const SpatialVect &u, const SpatialVect &v);
  void sub(const SpatialVect &v);
  void scale(const SpatialVect &v, Real s);
  void scale(Real s);

  // SpatialVect constants

  static const SpatialVect ZERO;
};


///////////////////////////////////////////////////////////////////////////////
//
//  class SpatialMat
//
//  SpatialMat are 6 x 6 matrices used for multibody dynamics
//  algorithms.
//
///////////////////////////////////////////////////////////////////////////////


class SpatialMat {

private:

  Mat3 _A, _B, _C, _D;

public:

  // constructors

  SpatialMat() {}

  SpatialMat(const Mat3 &A, const Mat3 &B, const Mat3 &C, const Mat3 &D)
  {_A = A; _B = B; _C = C; _D = D;}
  
  // accessors

  const Mat3 &A() const { return _A; }
        Mat3 &A()       { return _A; }
  const Mat3 &B() const { return _B; }
        Mat3 &B()       { return _B; }
  const Mat3 &C() const { return _C; }
        Mat3 &C()       { return _C; }
  const Mat3 &D() const { return _D; }
        Mat3 &D()       { return _D; }

  // input / output

  // friend ostream& operator<<(ostream &os, const SpatialMat &M);

  // other methods

  void inertiaInvert(const SpatialMat &M);
  void add(const SpatialMat &M, const SpatialMat &N);
  void add(const SpatialMat &M);
  void sub(const SpatialMat &M, const SpatialMat &N);
  void sub(const SpatialMat &M);
  void scale(const SpatialMat &M, Real s);
  void scale(Real s);

  void xform(const SpatialVect &u, SpatialVect &v) const;

  static const SpatialMat ZERO;

};

} // namespace mrt


#endif  // SPATIAL_H
