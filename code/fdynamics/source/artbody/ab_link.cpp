// Mirtich
#include "core/Misc.h"
#include "core/Link.h"
// This system
#include "ab_link.h"

namespace ab 
{

LinkRenderParams LinkParams::toRenderParams(void) const
{
  LinkRenderParams renderParams;

  renderParams.vert1 = Vect4::Zero;    
  renderParams.vert2 = dir * lenght;

  return renderParams;
}

LinkPhysParams LinkParams::toPhysParams(void) const 
{
  LinkPhysParams physParams;

  physParams.inertia = inertia;
  physParams.mass    = mass;
  physParams.lenght  = lenght;

  return physParams;
}

LinkParams::LinkParams():
dir(Vect4::UnitYNeg),
jointAxis(Vect4::UnitZ)
{
    generalized_coord = generalized_coord_p = generalized_coord_p_p = 0;
    springK = dumpK = 0;
}

Link::Link(const LinkParams& params_) :
_params(params_),
_localMatr(params_.modelMatr)
{
}

Link::~Link()
{
}

void Link::setModelMatr(const Matr4& modelMatr_)
{
  // calc previous modelMatr^-1
  Matr4 invPreModelMatr = _params.modelMatr;
  invPreModelMatr.invert();

  // calc new localMatr
  _localMatr.transform(invPreModelMatr, false);
  _localMatr.transform(modelMatr_, false);

  // copy new modelMatr
  _params.modelMatr = modelMatr_;
}

void Link::setLocalMatr(const Matr4& localMatr_)
{
  // copy new localMatr
  _localMatr = localMatr_;
}

void Link::updateState(Real pos_, Real vel_, Real acc_)
{
  _params.generalized_coord     = pos_;
  _params.generalized_coord_p   = vel_;
  _params.generalized_coord_p_p = acc_;
}

void Link::setInitialState(Real qd, Real qdd, Vect4 lin_v, Vect4 ang_v, Vect4 lin_acc, Vect4 ang_acc)
{
    _params.generalized_coord_p   = qd;
    _params.generalized_coord_p_p = qdd;
    _params.base_lin_vel = lin_v;
    _params.base_ang_vel = ang_v;
    _params.base_lin_acc = lin_acc;
    _params.base_ang_acc = ang_acc;
}


//
// Private
//
Matr4 Link::MatrLocalFromMirtCoord(const Link * link, const Matr4& matr)
{   
  Matr4 outMatr(matr);
  outMatr.translate(link->_params.centerMassPos, false);
  return outMatr;
}

Matr4 Link::MatrLocalToMirtCoord(const Link * link, const Matr4& matr)
{      
  Matr4 outMatr(matr);
  outMatr.translate(link->_params.centerMassPos * -1, false);
  return outMatr;
}

Matr4 Link::MatrGlobalFromMirtCoord(const Link * link, const Matr4& matr)
{
  Matr4 outMatr(matr);
  outMatr.translate(link->_params.centerMassPos * -1);
  return outMatr;
}

Vect4 Link::VectLocalFromMirtCoord(const Link * link, const Vect4& vect)
{      

  Matr4 rotLT = link->_localMatr;
  rotLT.setTranslate(Vect4::Zero);
  rotLT.invert();
  return rotLT * (vect + link->_params.centerMassPos);
}

Vect4 Link::VectLocalToMirtCoord(const Link * link, const Vect4& vect)
{      
  return (vect - link->_params.centerMassPos);
}

} // end of namespace ab