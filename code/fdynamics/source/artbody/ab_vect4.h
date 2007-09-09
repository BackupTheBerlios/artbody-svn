#ifndef _AB_VECT4_H_
#define _AB_VECT4_H_

// common
#include <math.h>
// this system
#include "ab_types.h"

namespace ab
{

class Vect4 
{
public:
  // constructors
  Vect4(void);
  Vect4(const Vect4& vect_);
  Vect4(Real x_, Real y_, Real z_);

  void operator = (const Vect4& vect_);

  Real  operator[] (int i) const;
  Real& operator[] (int i);

  Vect4  operator +     (const Vect4& vect_) const;
  Vect4  operator -     (const Vect4& vect_) const;
  Vect4  operator *     (Real scale)        const;
  Real   dot            (const Vect4& vect_) const;
  Vect4  cross          (const Vect4& vect_) const;
  Vect4  getPerpVector  (void)               const;

  Real   norm       (void) const;
  Real   norm2      (void) const;

  Real   angle      (const Vect4 vect_) const;

  void   negate     (void);
  void   normalize  (void);
  void   scale      (Real s);
  void   add        (const Vect4& vect_);
  void   sub        (const Vect4& vect_);

  static Vect4 Zero;
  static Vect4 UnitX;
  static Vect4 UnitY;
  static Vect4 UnitZ;
  static Vect4 UnitXNeg;
  static Vect4 UnitYNeg;
  static Vect4 UnitZNeg;

  // private:
  Real x, y, z, w;
};

inline bool isZero(Real f) { return (fabs(f) > Epsilon) ? false : true; }
inline bool isZero(Vect4 f) { return (fabs(f.x) + fabs(f.y) + fabs(f.z) > Epsilon) ? false : true; }

} // end of namespace ab

#endif //_AB_VECT4_H_