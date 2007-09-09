#ifndef _AB_LINK_H_
#define _AB_LINK_H_

// STL
#include <vector>
// This system
#include "ab_types.h"
#include "ab_vect4.h"
#include "ab_matr4.h"

namespace ab 
{

struct LinkRenderParams 
{
  Vect4 vert1;
  Vect4 vert2;
  Matr4 matrLocal;  // to parent
  Matr4 matrGlobal; // to world
  int   parentIdx;  // or -1 if no parents
};

struct LinkPhysParams 
{   
  Real  lenght;
  Real  mass;
  Vect4 inertia;
};

struct LinkParams 
{
    LinkParams();

    // initial data
    ab::Real   lenght;

    ab::Real   mass;
    ab::Vect4  inertia;

    ab::Matr4  modelMatr;     // center in inboard joint, rotation always around Z-axis
    ab::Vect4  centerMassPos; // in local coord
    ab::Vect4  rel_cm_pos;

    ab::Vect4  jointAxis;    // direction of inboard joint in local coordinates. Position of joint in local coord: always == (0, 0, 0)    

    ab::Vect4  dir;           // for render
    
    ab::Real   springK;
    ab::Real   dumpK;

    // for base (6-DOFs link)
    ab::Vect4  base_lin_vel;
    ab::Vect4  base_ang_vel;
    ab::Vect4  base_lin_acc;
    ab::Vect4  base_ang_acc;

    // real-time data
    ab::Real   generalized_coord;
    ab::Real   generalized_coord_p;
    ab::Real   generalized_coord_p_p;

    // helper function
    LinkRenderParams toRenderParams(void) const;
    LinkPhysParams  toPhysParams  (void) const;
};

class Link
{
friend class Joint;
public:
  explicit Link(const LinkParams& params_);
  virtual ~Link();

  const Matr4&      getLocalMatr(void) const { return _localMatr; }
  const LinkParams& getParams(void)    const { return _params; }
  const Matr4&      getModelMatr(void) const { return _params.modelMatr; }

  void              setInitialState(Real qd = 0.0, Real qdd = 0.0, 
                                    Vect4 lin_v = Vect4::Zero, 
                                    Vect4 ang_v = Vect4::Zero, 
                                    Vect4 lin_acc = Vect4::Zero, 
                                    Vect4 ang_acc = Vect4::Zero);


  void              setModelMatr(const Matr4& modelMatr_);
  void              setLocalMatr(const Matr4& localMatr_);

  void              updateState (Real pos_, Real vel_, Real acc_);
  void              updateEnergy(Real potE, Real kinE, bool bFirstFrame);

  static Matr4  MatrLocalToMirtCoord   (const Link * link, const Matr4& matr);
  static Matr4  MatrLocalFromMirtCoord (const Link * link, const Matr4& matr);
  static Matr4  MatrGlobalFromMirtCoord(const Link * link, const Matr4& matr);
  static Vect4  VectLocalToMirtCoord   (const Link * link, const Vect4& vect);
  static Vect4  VectLocalFromMirtCoord (const Link * link, const Vect4& vect);

private:
  LinkParams _params;
  Matr4  _localMatr;
  Real   _potE;
  Real   _kinE;

  Real   _springOffset;
};

} // end of namespace ab

#endif //_AB_LINK_H_
