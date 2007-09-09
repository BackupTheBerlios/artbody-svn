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

#ifndef MULTI_BODY_H
#define MULTI_BODY_H

//#ifndef DEPEND_IGNORE
#include <list>
#include <vector>
//#endif // DEPEND_IGNORE

#include <math.h>

#include "MatX.h"
#include "Link.h"
#include "OdeSolver.h"

namespace mrt {

class OdeSolver;

///////////////////////////////////////////////////////////////////////////////
//  class MultiBody
///////////////////////////////////////////////////////////////////////////////

class MultiBody
{
public:
  
  // constructor / destructor

  MultiBody(int numLinks);
  ~MultiBody();

  // accessing the links of the multibody
  int   getNumLinks() const         { return (int)_links.size(); }
  Link& getLink(int index)          { return _links[index]; }
  
  // access an element of the simulation state vector
  Real& getState(int index)         { return _state[index]; }

  // get or set the gravity vector defined for this multibody.
  // The default value is Vect3(0, -9.81, 0).
  const Vect3& getGravity() const   { return _gravity; }
  void setGravity(const Vect3 &newGravity) { _gravity = newGravity; }

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
  // frame; pass 0 (zero or NULL) in as the inboard link for this case.
  void connect(Link::JointType jointType, 
	       Link* pInboardLink,
	       const Vect3 &inLoc,
	       const Vect3 &inAxis,
	       const Vect3 &inRef,
	       Link* pOutboardLink,
	       const Vect3 &outLoc,
	       const Vect3 &outAxis,
	       const Vect3 &outRef);

  // step the simulation forward in time, from the initial time 't0'
  // to the final time 'tf'.
  void evolve(Real t0, Real tf);

  // update all poses for each link of the multibody.  After this
  // is executed, the 'pose()' method of a given link may be called.
  void updatePoses() {_links.front().updatePoses();}

  // Set integrator type
  void setIntegrator(OdeSolver::TYPE type) { _odeint->setType(type); }  

private:

 // Methods
  static void _derivs    (Real t, const Real sv[], Real dv[], const void *entity);
  static void _copy_state(Real sv[], Real dv[], const void *entity);

 // Data
  std::vector<Link> _links;
  std::vector<Real> _state;

  Vect3      _gravity;
  OdeSolver* _odeint;
};

} // namespace mrt


#endif  // #ifndef MULTI_BODY_H

