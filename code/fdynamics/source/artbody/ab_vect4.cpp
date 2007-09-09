// standard
#include <math.h>
// this system
#include "ab_vect4.h"

namespace ab
{

Vect4 Vect4::Zero;
Vect4 Vect4::UnitX(1.f, 0.f, 0.f);
Vect4 Vect4::UnitY(0.f, 1.f, 0.f);
Vect4 Vect4::UnitZ(0.f, 0.f, 1.f);
Vect4 Vect4::UnitXNeg(-1.f, 0.f, 0.f);
Vect4 Vect4::UnitYNeg(0.f, -1.f, 0.f);
Vect4 Vect4::UnitZNeg(0.f, 0.f, -1.f);

Vect4::Vect4(void)
{
   x = y = z = 0;
   w = 1;
}

Vect4::Vect4(const Vect4& vect_)
{
   x = vect_.x;
   y = vect_.y;
   z = vect_.z;
   w = vect_.w;
}

Vect4::Vect4(Real x_, Real y_, Real z_)
{
   x = x_;
   y = y_;
   z = z_;
   w = 1;
}


void Vect4::operator=(const Vect4& vect_)
{
   x = vect_.x;
   y = vect_.y;
   z = vect_.z;
   w = vect_.w;
}

Real Vect4::operator[] (int i) const
{
   return *((Real*)&x + i);
}

Real& Vect4::operator[] (int i)
{
   return *((Real*)&x + i);
}


Vect4 Vect4::operator+(const Vect4& vect_) const 
{
   Vect4 summ;

   summ.x = x + vect_.x;
   summ.y = y + vect_.y;
   summ.z = z + vect_.z;

   return summ;
}

Vect4 Vect4::operator-(const Vect4& vect_) const
{
   Vect4 diff;

   diff.x = x - vect_.x;
   diff.y = y - vect_.y;
   diff.z = z - vect_.z;

   return diff;
}

Vect4 Vect4::operator * (Real scale) const
{
   Vect4 prod;

   prod.x = x * scale;
   prod.y = y * scale;
   prod.z = z * scale;
   
   return prod;
}

Real Vect4::dot(const Vect4& vect_) const
{
   return (x * vect_.x) + (y * vect_.y) + (z * vect_.z);
}

Vect4 Vect4::cross(const Vect4& vect_) const
{
   Vect4 product;

   product.x = y * vect_.z - z * vect_.y;
   product.y = z * vect_.x - x * vect_.z;
   product.z = x * vect_.y - y * vect_.x;

   return product;
}

Vect4  Vect4::getPerpVector(void) const
{
    Vect4 perp(0, -z, y);
    if (isZero(z)) {
        if (isZero(y)) {
            perp = Vect4(0, -x, 0);
        } else {
            perp = Vect4(-y, x, 0);
        }

    }

    return perp;
}

Real Vect4::angle(const Vect4 vect_) const
{
    Vect4 first(*this);
    Vect4 second(vect_);

    first.normalize();
    second.normalize();
    Real angle_cos = first.dot(second);
    return acos(angle_cos);
}

Real Vect4::norm(void) const
{
   return sqrt(x * x + y * y + z * z);
}

Real Vect4::norm2() const 
{
   return x * x + y * y + z * z;
}

void Vect4::negate(void)
{
   x *= -1;
   y *= -1;
   z *= -1;
}

void Vect4::normalize(void)
{
   Real s = 1 / norm();
   x *= s;
   y *= s;
   z *= s;
}

void Vect4::scale(Real s)
{
   x *= s;
   y *= s;
   z *= s;
}

void Vect4::add(const Vect4& vect_)
{
   x += vect_.x;
   y += vect_.y;
   z += vect_.z;
}

void Vect4::sub(const Vect4& vect_)
{
   x -= vect_.x;
   y -= vect_.y;
   z -= vect_.z;
}

} // end of namespace ab