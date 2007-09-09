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

#ifndef MATX_H
#define MATX_H

//#ifndef DEPEND_IGNORE
//#include <iostream.h>
//#if INVENTOR
//#include <Inventor/SbLinear.h>
//#endif
//#endif //DEPEND_IGNORE

#include <math.h>

#include "RealType.h"
#include "Mat3.h"

namespace mrt {

class Se3;

///////////////////////////////////////////////////////////////////////////////
//
//  class MatX
//
///////////////////////////////////////////////////////////////////////////////


class MatX {

  friend class Se3;

private:

  Mat3 R;
  Vect3 d;

public:

  // constructors //////////////////////////////////////////////////////////////

  MatX()                                {}
  MatX(const Mat3 &R_, const Vect3 &d_) {set(R_, d_);}
  MatX(const Se3 &T)                    {set(T);}


  // setters / accessors / translators /////////////////////////////////////////
  
  inline void set(const Mat3 &R_, const Vect3 &d_) {R = R_; d = d_;}
         void set(const Se3 &T);

  const Mat3  &rot()   const {return R;}
  const Vect3 &trans() const {return d;}
        Mat3  &rot()         {return R;}
        Vect3 &trans()       {return d;}
  
  // input / output ////////////////////////////////////////////////////////////

//  inline ostream& print(ostream &os) const;
//         istream& read(istream &is);

  // operations returning result via this //////////////////////////////////////

  // The operation result indicated by the comments is always returned
  // in this.  The symbol [!] indicates that this must be distinct
  // from all of the operands.

  void mult(const MatX &M, const MatX &N);    // M * N     [!]
  void premult(const MatX &M);                // M * this  [!]
  void postmult(const MatX &M);               // this * M  [!]
  void invert(const MatX &M);                 // M^-1      [!]
  void invert();                              // this^-1

  // Transforming Vect3s ///////////////////////////////////////////////////////

  // MatXs can transform elements of R^3 either as vectors or as
  // points.  The [!] indicates that the operands must be distinct.

  inline void xformVect(const Vect3 &v, Vect3 &xv) const; // this*(v 0)=>xv  [!]
  inline void xformVect(Vect3 &v) const;                  // this*(v 0)=>v
  inline void xformPoint(const Vect3 &p, Vect3 &xp) const;// this*(p 1)=>xp  [!]
  inline void xformPoint(Vect3 &p) const;                 // this*(p 1)=>p

  // These are exactly like the above methods, except the inverse
  // transform this^-1 is used.
  inline void invXformVect(const Vect3 &v, Vect3 &xv) const;
  inline void invXformVect(Vect3 &v) const;                 
  inline void invXformPoint(const Vect3 &p, Vect3 &xp) const;
  inline void invXformPoint(Vect3 &p) const;                 


  // MatX constants ////////////////////////////////////////////////////////////

  static const MatX ID;      // identity matrix

};


///////////////////////////////////////////////////////////////////////////////
//
//  inline function definitions
//
///////////////////////////////////////////////////////////////////////////////

//ostream& MatX::print(ostream &os) const
//{
//  return os << R << d << endl;
//}

void MatX::xformVect(const Vect3 &v, Vect3 &xv) const
{
  R.xform(v, xv);
}

  
void MatX::xformVect(Vect3 &v) const
{
  R.xform(v);
}

  
void MatX::xformPoint(const Vect3 &p, Vect3 &xp) const
{
  R.xform(p, xp);
  xp.add(d);
}


void MatX::xformPoint(Vect3 &p) const
{
  R.xform(p);
  p.add(d);
}


void MatX::invXformVect(const Vect3 &v, Vect3 &xv) const
{
  R.invXform(v, xv);
}

  
void MatX::invXformVect(Vect3 &v) const
{
  R.invXform(v);
}

  
void MatX::invXformPoint(const Vect3 &p, Vect3 &xp) const
{
  xp.sub(p, d);
  R.invXform(xp);
}


void MatX::invXformPoint(Vect3 &p) const
{
  p.sub(d);
  R.invXform(p);
}

///////////////////////////////////////////////////////////////////////////////
//
//  stream operators
//
///////////////////////////////////////////////////////////////////////////////

//inline ostream &operator<<(ostream &os, const MatX &X)  {return X.print(os);}
//inline istream &operator>>(istream &is, MatX &X)        {return X.read(is);}

} // namespace mrt

#endif  // MATX_H
