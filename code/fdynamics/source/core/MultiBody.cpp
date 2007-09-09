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

#include "Misc.h"
#include "MultiBody.h"
#include "OdeSolver.h"

using namespace mrt;

// the default gravity vector                          
static const Vect3 DEFAULT_GRAVITY(0.0, -9.81, 0.0);
//static const Vect3 DEFAULT_GRAVITY(0.0, -0.981, 0.0);
//static const Vect3 DEFAULT_GRAVITY(0.0, 0.0, 0.0);


///////////////////////////////////////////////////////////////////////////////
//  local helper functions
///////////////////////////////////////////////////////////////////////////////


// If a vector v (assumed to be of unit magnitude!) is close to one of
// the +ve or -ve axes-aligned vectors, set it exactly equal to that
// vector.  Also return a result indicating the _dirType of v.
Link::DirType canonicalize(Vect3 &v)
{
#define UNITY (1.0 - 1.0e-6)
  if      (v.x >=  UNITY) {v = Vect3::I ; return Link::X ;} 
  else if (v.x <= -UNITY) {v = Vect3::I_; return Link::X_;} 
  else if (v.y >=  UNITY) {v = Vect3::J ; return Link::Y ;} 
  else if (v.y <= -UNITY) {v = Vect3::J_; return Link::Y_;} 
  else if (v.z >=  UNITY) {v = Vect3::K ; return Link::Z ;} 
  else if (v.z <= -UNITY) {v = Vect3::K_; return Link::Z_;} 
  else return Link::GEN;
#undef UNITY
}  


// Return code (0-23) of an aligned rotation matrix.  N.B(). this should
// only be called if R is known to be axes aligned!  I.e., each column
// and each row of R should have a 1 and two 0's, although this
// routine is robust to small floating point errors.

int rotCode(const Mat3 &R)
{
#define ALMOST_ONE 0.9  
  int i, j;
  Vect3 axis, ref;
  Vect3 dirs[6] = 
        {Vect3::I, Vect3::I_, Vect3::J, Vect3::J_, Vect3::K , Vect3::K_};

  axis = R.zcol();
  ref  = R.xcol();
  for (i = 0; axis.dot(dirs[i]) < ALMOST_ONE; i++);
  for (j = 0; ref.dot(dirs[((i & ~1) + 2 + j) % 6]) < ALMOST_ONE; j++);
  return i * 4 + j;

#undef ALMOST_ONE
}



MatX jointFrame(const Vect3 &loc, const Vect3 &axis_, const Vect3 &ref_)
{
  Vect3 axis, ref, other;
  Mat3 R;

  axis.normalize(axis_);
  ref.displace(ref_, axis, - ref_.dot(axis));  // in case ref & axis not perp
  ref.normalize(ref);
  other.cross(axis, ref);

  R.setXcol(ref);
  R.setYcol(other);
  R.setZcol(axis);

  return MatX(R, loc);
}



///////////////////////////////////////////////////////////////////////////////
//  class MultiBody
///////////////////////////////////////////////////////////////////////////////

MultiBody::MultiBody(int numLinks) : _links(numLinks, Link())
{
  _odeint = new OdeSolver();

  _gravity = DEFAULT_GRAVITY;

  int i = 0;
  std::vector<Link>::iterator link;

  FOR_EACH(_links, link) {link->mb = this; link->_idx = i++;}
}


MultiBody::~MultiBody()
{
  if (_odeint)
    delete _odeint;
}



void MultiBody::_derivs(
  Real t,              // time			     
  const Real sv[],     // state vector
  Real dv[],           // derivative vector
  const void *entity)  // MultiBody being integrated
{
  MultiBody *mb;
  SpatialVect dummyVect;
  SpatialMat dummyMat;

  mb = (MultiBody *) entity;

  mb->_links.front().featherstonePass1(sv, MatX::ID, Vect3::ZERO, Vect3::ZERO,
				      dummyMat, dummyVect);

  mb->_links.front().featherstonePass2(sv, dv, SpatialVect::ZERO);         
}

void MultiBody::_copy_state(Real sv[], Real dv[], const void *entity)
{
    return;

    MultiBody *mb;
    mb = (MultiBody *) entity;

    std::vector<Link>::iterator iter;
    for (iter = mb->_links.begin(); iter != mb->_links.end(); iter++) {
        Link& lnk = *iter;
        dv[lnk._svIdx] = sv[lnk._svIdx + 1];
    }
    return;
}


void MultiBody::evolve(Real t0, Real tf) 
{
  std::vector<Real> newState(_state);

  _odeint->integrate(&_state[0], &newState[0], int(_state.size()), t0, tf,
                     _derivs, _copy_state, this);

  _state = newState;
}
    
// Connect two links of a multibody.  For each link, we specify: (1)
// loc, the location of the joint; (2) axis, the direction of the
// joint axis, and (3) ref, the direction of a reference axis.  All
// quantities are specified in the body frame of the corresponding
// link.  This routine connects the two links, such that the axes
// coincide.  For prismatic joints, the ref axes are always in the
// same direction, and the locs coincide when the joint is in the "0"
// configuration.  For revolute joints, the locs always coincide, and
// the ref axes are in the same direction when the joint is in the "0"
// configuration.  The base link should be connected to the inertial
// frame; pass 0 in as the inboard link for this case.

void MultiBody::connect(Link::JointType jt, 
			Link* inboard,
			const Vect3 &inLoc,
			const Vect3 &inAxis,
			const Vect3 &inRef,
			Link* outboard,
			const Vect3 &outLoc,
			const Vect3 &outAxis,
			const Vect3 &outRef)
{
  Vect3 pdir, pref, ref, v;
  Se3 Tji, Tjo;
  MatX X;

  // check inboard and outboard parameters
  if (outboard->mb != this)
    crash("MultiBody::connect(): outboard Link not found in Multibody");
  if (inboard && inboard->mb != this)
    crash("MultiBody::connect(): inboard Link not found in Multibody");
  if (jt == Link::FLT && outboard != &_links.front())
    crash("MultiBody::connect(): only base link can have floating joint");
  if (!inboard && outboard != &_links.front())
    crash("MultiBody::connect(): only base link connectable to inertial frame");
  if (inboard && outboard == &_links.front())
    crash("MultiBody::connect(): base link must connect to inertial frame");
  if (outboard->_jointType != Link::NONE) 
    crash("MultiBody::connect(): outboard link already connected");

  outboard->inboard = inboard;
  if (inboard) inboard->_outboards.push_back(outboard);

  X = jointFrame(inLoc, inAxis, inRef);
  outboard->_Xij.invert(X);
  outboard->_Xjo = jointFrame(outLoc, outAxis, outRef);
 
  outboard->_dir = outboard->_Xjo.rot().zcol();
  outboard->_dirType = canonicalize(outboard->_dir);

  outboard->_jointType = jt;

  // Compile joint.  For both prismatic and revolute joints we
  // compute _Xf0.  For revolute joints only, we computed _mom, _mom2,
  // halfrotCode and fullRotCode.

  switch(outboard->_jointType) {
    case Link::PRI:
        outboard->_Xf0.mult(outboard->_Xjo, outboard->_Xij);
        break;
    case Link::REV:
        ref = outboard->_Xjo.rot().xcol();
        pref = outboard->_Xij.rot().xcol();
        pdir = outboard->_Xij.rot().zcol();
        v.displace(outboard->_Xjo.trans(), outboard->_dir, outboard->_Xij.trans().z);
        outboard->_Xf0.set(Mat3::ZERO, v);
        outboard->_mom.cross(outboard->_Xjo.trans(), outboard->_dir);
        outboard->_mom2.cross(outboard->_dir, outboard->_mom);
        if (outboard->_dirType == Link::GEN || canonicalize(ref) == Link::GEN ||
            canonicalize(pdir) == Link::GEN || canonicalize(pref) == Link::GEN)
            outboard->halfRotCode = outboard->fullRotCode = -1;
        else {
            outboard->halfRotCode = rotCode(outboard->_Xjo.rot());
            outboard->fullRotCode = rotCode(outboard->_Xij.rot()) + 24 * outboard->halfRotCode;
        }
    default:
        break;
  }

  // initialize svIdx, joint of outboard link
  outboard->_svIdx = int(_state.size());
  switch(outboard->_jointType) {
    case Link::REV:
    case Link::PRI:
        _state.insert(_state.end(), 2, 0.0);
        break;
    case Link::FLT:
        {        
            _state.insert(_state.end(), 13, 0.0);
            RBConfig * rbConf = ((RBConfig *) &*(_state.end() - 13));
            rbConf->Tbi.set(Se3(Quat::getIDQuat(), inLoc));
        }
        break;
  }

  Real* st = &_state[0];

}


