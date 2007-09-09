#ifndef _AB_MATR4_H_
#define _AB_MATR4_H_

// this system
#include "ab_types.h"
#include "ab_vect4.h"

namespace ab
{

class Matr4 
{
public:
  enum AxisType 
  {
    Axis_X,
    Axis_Y,
    Axis_Z
  };

  Matr4();
  Matr4(const Matr4& matr_);
  explicit Matr4(const Vect4& pos);
  Matr4(const Vect4& rotate, const Vect4& translate, const Vect4& scale);
  Matr4(const Vect4& xAxis, const Vect4& yAxis, const Vect4& zAxis, const Vect4& translate);

  void operator = (const Matr4& matr_);

  Matr4  operator + (const Matr4& matr_) const;
  Matr4  operator - (const Matr4& matr_) const;
  Matr4  operator * (const Matr4& matr_) const;
  Vect4  operator * (const Vect4& vect_) const;

  Vect4  transformAsVector(const Vect4& vect_) const;
  Vect4  transformAsPoint (const Vect4& vect_) const;

  void  transform(const Matr4& matr_, bool preTransform = true);

  void  translate     (const Vect4& trans, bool preTransform = true);
  void  translateLocal(AxisType dir, Real trans, bool preTransform = true);
  void  scale         (const Vect4& sc,    bool preTransform = true);
  void  rotateX       (Real rotateAngleRad,  bool preTransform = true);
  void  rotateY       (Real rotateAngleRad,  bool preTransform = true);
  void  rotateZ       (Real rotateAngleRad,  bool preTransform = true);
  void  transpose     (void);
  void  invert        (void);

  Real det      (void) const;
  Real minor    (int row, int col) const;

  Vect4 getTranslate(void) const;
  void  setTranslate(const Vect4& translate);

  Vect4 getAxisX(void) const;
  Vect4 getAxisY(void) const;
  Vect4 getAxisZ(void) const;

  void  setAxisX(const Vect4& xAxis);
  void  setAxisY(const Vect4& yAxis);
  void  setAxisZ(const Vect4& zAxis);

  Real  elem(int row, int col) const;
  Real& elem(int row, int col);

  static Matr4 Identity;

private:
  Real _elems[4][4];
};

} // end of namespace ab

#endif //_AB_MATR4_H_