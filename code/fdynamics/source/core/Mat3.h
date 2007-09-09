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

#ifndef MAT3_H
#define MAT3_H

//#ifndef DEPEND_IGNORE
//#include <iostream.h>
//#if INVENTOR
//#include <Inventor/SbLinear.h>
//#endif
//#endif //DEPEND_IGNORE

#include <math.h>

#include "RealType.h"
#include "Vect3.h"
#include "Quat.h"

namespace mrt {

///////////////////////////////////////////////////////////////////////////////
//
//  class Mat3
//
///////////////////////////////////////////////////////////////////////////////


class Mat3 {

  friend class Quat;
  friend class MatX;

private:

  // (stored in row-major order)
  Real xx, xy, xz,
       yx, yy, yz,
       zx, zy, zz;

public:

  // constructors //////////////////////////////////////////////////////////////

  Mat3() {}
  Mat3(const Vect3 &diag, const Vect3 &sym) {set(diag, sym);}
  Mat3(const Vect3 &axis, Real angle, int normalizeAxis = 1)
    {set(axis, angle, normalizeAxis);}
  Mat3(const Quat &q) {set(q);}


  // setters / accessors ///////////////////////////////////////////////////////

  // make a symmetric matrix, given the diagonal and symmetric
  // (off-diagonal) elements in canonical order
  inline void set(const Vect3 &diag, const Vect3 &sym);
  // set Mat3 as a rotation of 'angle' radians about 'axis'
  // axis is automatically normalized unless normalizeAxis = 0
  inline void set(const Vect3 &axis, Real angle, int normalizeAxis = 1);
  void set(const Quat &q);

  // index-based access:  0=xrow, 1=yrow, 2=zrow.
  const Vect3 &operator[](int i) const {return *(((Vect3 *) &xx) + i);}
        Vect3 &operator[](int i)       {return *(((Vect3 *) &xx) + i);}

  // set matrix to the skew symmetric matrix corresponding to 'v X'
  inline void setSkew(const Vect3 &v);

  // for reading rows
  const Vect3 &xrow() const {return *((Vect3 *) &xx);}
  const Vect3 &yrow() const {return *((Vect3 *) &yx);}
  const Vect3 &zrow() const {return *((Vect3 *) &zx);}
  // for writing to rows
  Vect3 &xrow()  {return *((Vect3 *) &xx);}
  Vect3 &yrow()  {return *((Vect3 *) &yx);}
  Vect3 &zrow()  {return *((Vect3 *) &zx);}

  // for reading columns
  Vect3 xcol() const {return Vect3(xx, yx, zx);}
  Vect3 ycol() const {return Vect3(xy, yy, zy);}
  Vect3 zcol() const {return Vect3(xz, yz, zz);}
  // for writing to columns
  inline void setXcol(const Vect3 &v);
  inline void setYcol(const Vect3 &v);
  inline void setZcol(const Vect3 &v);

  // for reading a symmetric matrix
  Vect3 diag() const {return Vect3(xx, yy, zz);}
  Vect3 sym()  const {return Vect3(yz, zx, xy);}


  // input / output ////////////////////////////////////////////////////////////

//  ostream& print(ostream &os) const;

  // operations not returning a Mat3 ///////////////////////////////////////////

  inline Real det() const;

  // operations returning result via this //////////////////////////////////////

  // The operation result indicated by the comments is always returned
  // in this.  The symbol [!] indicates that this must be distinct
  // from all of the operands.  The invert() methods are based on the
  // explicit inversion formula from determinants; there are faster
  // and more accurate ways.  The invert() methods return one if the
  // matrix was not invertible, otherwise zero.

  inline void xpose(const Mat3 &M);                   // M^T       [!]
  inline void xpose();                                // this^T
  inline void symmetrize(const Mat3 &M);              // M + M^T
  inline void symmetrize();                           // this + this^T
         int  invert(const Mat3 &M);                  // M^-1      [!]
         int  invert();                               // this^-1
  inline void negate(const Mat3 &M);                  // -M
  inline void negate();                               // -this
  inline void add(const Mat3 &M, const Mat3 &N);      // M + N
  inline void add(const Mat3 &M);                     // this + M
  inline void sub(const Mat3 &M, const Mat3 &N);      // M - N
  inline void sub(const Mat3 &M);                     // this - M
  inline void scale(const Mat3 &M, Real s);           // s * M
  inline void scale(Real s);                          // s * this
         void mult(const Mat3 &M, const Mat3 &N);     // M * N     [!]
         void premult(const Mat3 &M);                 // M * this  [!]
         void postmult(const Mat3 &M);                // this * M  [!]

  // Transforming Vect3s ///////////////////////////////////////////////////////

  inline void xform(const Vect3 &v, Vect3 &xv) const; // (this)(v) => xv; 
                                                      // v & xv must be distinct
  inline void xform(Vect3 &v) const;                  // (this)(v) => v

  // These are exactly like the above methods, except the inverse
  // transform this^-1 (= this^T) is used.  This can be thought of as
  // a row vector transformation, e.g.: (v^T)(this) => xv^T
  inline void invXform(const Vect3 &v, Vect3 &xv) const;
  inline void invXform(Vect3 &v) const;


  // Mat3 constants ////////////////////////////////////////////////////////////

  static const Mat3 ZERO;    // zero matrix
  static const Mat3 ID;      // identity matrix

};

///////////////////////////////////////////////////////////////////////////////
//
//  inline function definitions
//
///////////////////////////////////////////////////////////////////////////////

void Mat3::set(const Vect3 &diag, const Vect3 &sym)
{
  xx = diag.x;
  yy = diag.y;
  zz = diag.z;
  yz = zy = sym.x;
  zx = xz = sym.y;
  xy = yx = sym.z;
}


void Mat3::set(const Vect3 &axis, Real angle, int normalizeAxis)
{
  Quat q;

  q.set(angle, axis, normalizeAxis);
  set(q);
}

void Mat3::setXcol(const Vect3 &v)
{
  xx = v.x;
  yx = v.y;
  zx = v.z;
}


void Mat3::setYcol(const Vect3 &v)
{
  xy = v.x;
  yy = v.y;
  zy = v.z;
}


void Mat3::setZcol(const Vect3 &v)
{
  xz = v.x;
  yz = v.y;
  zz = v.z;
}


void Mat3::setSkew(const Vect3 &v)
{
  xx = yy = zz = 0.0;
  zy =  v.x;
  yz = -v.x;
  xz =  v.y;
  zx = -v.y;
  yx =  v.z;
  xy = -v.z;
}


Real Mat3::det() const
{
  return  xx * (yy * zz - yz * zy)
        + xy * (yz * zx - yx * zz)
        + xz * (yx * zy - yy * zx);
}


void Mat3::xpose(const Mat3 &M)
{
  xx = M.xx;
  xy = M.yx;
  xz = M.zx;

  yx = M.xy;
  yy = M.yy;
  yz = M.zy;

  zx = M.xz;
  zy = M.yz;
  zz = M.zz;
}


void Mat3::xpose()
{
  Real tmp;

  tmp = xy;
  xy = yx;
  yx = tmp;;

  tmp = yz;
  yz = zy;
  zy = tmp;

  tmp = zx;
  zx = xz;
  xz = tmp;
}


void Mat3::symmetrize(const Mat3 &M)
{
  xx = 2 * M.xx;
  yy = 2 * M.yy;
  zz = 2 * M.zz;
  xy = yx = M.xy + M.yx;
  yz = zy = M.yz + M.zy;
  zx = xz = M.zx + M.xz;
}


void Mat3::symmetrize()
{
  xx = 2 * xx;
  yy = 2 * yy;
  zz = 2 * zz;
  xy = yx = xy + yx;
  yz = zy = yz + zy;
  zx = xz = zx + xz;
}


void Mat3::negate(const Mat3 &M)
{
  xx = - M.xx;
  xy = - M.xy;
  xz = - M.xz;

  yx = - M.yx;
  yy = - M.yy;
  yz = - M.yz;

  zx = - M.zx;
  zy = - M.zy;
  zz = - M.zz;
}  


void Mat3::negate()
{
  xx = - xx;
  xy = - xy;
  xz = - xz;

  yx = - yx;
  yy = - yy;
  yz = - yz;

  zx = - zx;
  zy = - zy;
  zz = - zz;
}  


void Mat3::add(const Mat3 &M, const Mat3 &N)
{
  xx = M.xx + N.xx;
  xy = M.xy + N.xy;
  xz = M.xz + N.xz;

  yx = M.yx + N.yx;
  yy = M.yy + N.yy;
  yz = M.yz + N.yz;

  zx = M.zx + N.zx;
  zy = M.zy + N.zy;
  zz = M.zz + N.zz;
}


void Mat3::add(const Mat3 &M)
{
  xx += M.xx;
  xy += M.xy;
  xz += M.xz;

  yx += M.yx;
  yy += M.yy;
  yz += M.yz;

  zx += M.zx;
  zy += M.zy;
  zz += M.zz;
}


void Mat3::sub(const Mat3 &M, const Mat3 &N)
{
  xx = M.xx - N.xx;
  xy = M.xy - N.xy;
  xz = M.xz - N.xz;

  yx = M.yx - N.yx;
  yy = M.yy - N.yy;
  yz = M.yz - N.yz;

  zx = M.zx - N.zx;
  zy = M.zy - N.zy;
  zz = M.zz - N.zz;
}


void Mat3::sub(const Mat3 &M)
{
  xx -= M.xx;
  xy -= M.xy;
  xz -= M.xz;

  yx -= M.yx;
  yy -= M.yy;
  yz -= M.yz;

  zx -= M.zx;
  zy -= M.zy;
  zz -= M.zz;
}


void Mat3::scale(const Mat3 &M, Real s)
{
  xx = s * M.xx; 
  xy = s * M.xy; 
  xz = s * M.xz;
  yx = s * M.yx; 
  yy = s * M.yy; 
  yz = s * M.yz;
  zx = s * M.zx; 
  zy = s * M.zy; 
  zz = s * M.zz;
}


void Mat3::scale(Real s)
{
  xx *= s; 
  xy *= s; 
  xz *= s;
  yx *= s; 
  yy *= s; 
  yz *= s;
  zx *= s; 
  zy *= s; 
  zz *= s;
}


void Mat3::xform(const Vect3 &v, Vect3 &xv) const
{
  xv.x = xx * v.x + xy * v.y + xz * v.z;
  xv.y = yx * v.x + yy * v.y + yz * v.z;
  xv.z = zx * v.x + zy * v.y + zz * v.z;
}


void Mat3::xform(Vect3 &v) const
{
  Real ox, oy;

  ox = v.x; oy= v.y;
  v.x = xx * ox + xy * oy + xz * v.z;
  v.y = yx * ox + yy * oy + yz * v.z;
  v.z = zx * ox + zy * oy + zz * v.z;
}


void Mat3::invXform(const Vect3 &v, Vect3 &xv) const
{
  xv.x = xx * v.x + yx * v.y + zx * v.z;
  xv.y = xy * v.x + yy * v.y + zy * v.z;
  xv.z = xz * v.x + yz * v.y + zz * v.z;
}


void Mat3::invXform(Vect3 &v) const
{
  Real ox, oy;

  ox = v.x; oy= v.y;
  v.x = xx * ox + yx * oy + zx * v.z;
  v.y = xy * ox + yy * oy + zy * v.z;
  v.z = xz * ox + yz * oy + zz * v.z;
}

///////////////////////////////////////////////////////////////////////////////
//
//  stream operators
//
///////////////////////////////////////////////////////////////////////////////

//inline ostream &operator<<(ostream &os, const Mat3 &M)  {return M.print(os);}

} // namespace mrt

#endif  // MAT3_H
