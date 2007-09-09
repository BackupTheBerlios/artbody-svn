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

#ifndef SE3_H
#define SE3_H

//#ifndef DEPEND_IGNORE
//#include <iostream.h>
//#if INVENTOR
//#include <Inventor/SbLinear.h>
//#include <Inventor/nodes/SoTransform.h>
//#else
#include <string.h>
//#endif
//#endif //DEPEND_IGNORE

#include <math.h>

#include "RealType.h"
#include "MatX.h"

namespace mrt {

//
//  class Se3
//
///////////////////////////////////////////////////////////////////////////////


class Se3 {

  friend class MatX;

public:

  Quat q;     // rotation component
  Vect3 d;    // translation component

public:

  // constructors //////////////////////////////////////////////////////////////


  Se3() {}
  Se3(const Quat &q_, const Vect3 &d_) {set(q_, d_);}
  Se3(const MatX &X) {set(X);}

  // setters / accessors / translators /////////////////////////////////////////
  void set(const Se3 &another);
  void set(const Quat &q_, const Vect3 &d_);
  void set(const MatX &X) {q.set(X.R); d = X.d;}
#if INVENTOR
  inline void set(const SoTransform *T);
  inline void toSoTransform(SoTransform *T) const;
#endif

  const Quat  &rot()   const {return q;}
  const Vect3 &trans() const {return d;}
        Quat  &rot()         {return q;}
        Vect3 &trans()       {return d;}

  // input / output ////////////////////////////////////////////////////////////


  // inline ostream& print(ostream &os) const;

  // Read Se3 from input stream.  The Se3 specification must be
  // enclosed in curly brackets { }.  Se3's are built up from a
  // sequence of translation and rotation operations.  A translation
  // opereration is specified by "trans v", where v is the translation
  // vector.  A rotation operation is specified by: "rot a v", where a
  // is the scalar rotation in *degrees*, and v is the axis of
  // rotation (a vector).

  // istream& read(istream &is);

  // operations returning result via this //////////////////////////////////////

  // The operation result indicated by the comments is always returned
  // in this.  The symbol [!] indicates that this must be distinct
  // from all of the operands.

  inline void mult(const Se3 &T, const Se3 &U);    // T * U     [!]
  inline void premult(const Se3 &T);               // T * this  [!]
  inline void postmult(const Se3 &T);              // this * T  [!]
  inline void invert(const Se3 &T);                // T^-1
  inline void invert();                            // this^-1

  // Transforming Vect3s ///////////////////////////////////////////////////////

  // Se3s can transform elements of R^3 either as vectors or as
  // points.  Multiple operands need not be distinct.

  inline void xformVect(const Vect3 &v, Vect3 &xv) const;  // this * (v 0) => xv
  inline void xformVect(Vect3 &v) const;		   // this * (v 0) => v
  inline void xformPoint(const Vect3 &p, Vect3 &xp) const; // this * (p 1) => xp
  inline void xformPoint(Vect3 &p) const;		   // this * (p 1) => p

  // These are exactly like the above methods, except the inverse
  // transform this^-1 is used.
  inline void invXformVect(const Vect3 &v, Vect3 &xv) const;
  inline void invXformVect(Vect3 &v) const;                 
  inline void invXformPoint(const Vect3 &p, Vect3 &xp) const;
  inline void invXformPoint(Vect3 &p) const;                 


  // Se3 constants /////////////////////////////////////////////////////////////
  static const Se3 ID;      // identity Se3
  static const Se3& getIDSe3(void) { static Se3 se(Quat::getIDQuat(), Vect3::ZERO); return se; }
};



///////////////////////////////////////////////////////////////////////////////
//
//  inline function definitions
//
///////////////////////////////////////////////////////////////////////////////



#if INVENTOR
void Se3::set(const SoTransform *xform)
{
  const float *quat;
  const float *trans;

  quat = xform->rotation.getValue().getValue();
  q.x_ = quat[0]; q.y_ = quat[1]; q.z_ = quat[2]; q.s_ = quat[3];

  trans = xform->translation.getValue().getValue();
  d.x = trans[0];
  d.y = trans[1];
  d.z = trans[2];
}
#endif


#if INVENTOR
void Se3::toSoTransform(SoTransform *xform) const
{
#if OPCOUNTS
  xform->rotation.setValue
    (q.x_.toDouble(), q.y_.toDouble(), q.z_.toDouble(), q.s_.toDouble());
  xform->translation.setValue(d.x.toDouble(), d.y.toDouble(), d.z.toDouble());
#else
  xform->rotation.setValue(q.x_, q.y_, q.z_, q.s_);
  xform->translation.setValue(d.x, d.y, d.z);
#endif
}
#endif
  

//ostream& Se3::print(ostream &os) const
//{
//  return os << q << d;
//}


void Se3::mult(const Se3 &T, const Se3 &U)
{
  q.mult(T.q, U.q);
  T.q.xform(U.d, d);
  d.add(d, T.d);
}


void Se3::premult(const Se3 &T)
{
  q.premult(T.q);
  T.q.xform(d);
  d.add(T.d);
}


void Se3::postmult(const Se3 &T)
{
  Vect3 v;

  q.xform(T.d, v);
  d.add(v);
  q.postmult(T.q);
}


void Se3::invert(const Se3 &T)
{
  q.s_ = -T.q.s_;
  q.x_ =  T.q.x_;
  q.y_ =  T.q.y_;
  q.z_ =  T.q.z_;
  q.xform(T.d, d);
  d.negate(d);
}


void Se3::invert()
{
  q.s_ = -q.s_;
  q.xform(d);
  d.negate();
}


void Se3::xformVect(const Vect3 &v, Vect3 &xv) const
{
  q.xform(v, xv);
}


void Se3::xformVect(Vect3 &v) const
{
  q.xform(v);
}


void Se3::xformPoint(const Vect3 &p, Vect3 &xp) const
{
  q.xform(p, xp);
  xp.add(d);
}


void Se3::xformPoint(Vect3 &p) const
{
  q.xform(p);
  p.add(d);
}


void Se3::invXformVect(const Vect3 &v, Vect3 &xv) const
{
  q.invXform(v, xv);
}


void Se3::invXformVect(Vect3 &v) const
{
  q.invXform(v);
}


void Se3::invXformPoint(const Vect3 &p, Vect3 &xp) const
{
  xp.sub(p, d);
  q.invXform(xp);
}


void Se3::invXformPoint(Vect3 &p) const
{
  p.sub(d);
  q.invXform(p);
}

///////////////////////////////////////////////////////////////////////////////
//
//  stream operators
//
///////////////////////////////////////////////////////////////////////////////

//inline ostream &operator<<(ostream &os, const Se3 &T)   {return T.print(os);}
//inline istream &operator>>(istream &is, Se3 & T)        {return T.read(is);}

} // namespace mrt


#endif // SE3_H

