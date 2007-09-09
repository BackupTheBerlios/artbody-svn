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

#ifndef LINK_H
#define LINK_H

#include <math.h>


#include "RealType.h"

#include "Spatial.h"
#include "Se3.h"

namespace mrt {

    ///////////////////////////////////////////////////////////////////////////////
    //  struct LinkState
    ///////////////////////////////////////////////////////////////////////////////

    struct LinkState {
        Real pos;     // scalar position of joint
        Real vel;     // scalar velocity of joint
        Real acc;     // scalar acceleration of joint
    };

    ///////////////////////////////////////////////////////////////////////////////
    //  struct RBConfig
    ///////////////////////////////////////////////////////////////////////////////

    struct RBConfig {
        Se3 Tbi;
        Vect3 v;
        Vect3 w;
    };


    ///////////////////////////////////////////////////////////////////////////////
    //  class Link
    ///////////////////////////////////////////////////////////////////////////////

    class Link {

        friend class MultiBody;

    public:

        // Right now, only REV, PRI, and FLT joint types supported

        enum JointType {//FIX, // fixed attachement (0 DOF)
            REV, // revolute          (1 DOF)
            PRI, // prismatic         (1 DOF)
            //SCR, // screw             (1 DOF)
            //CYL, // cylindrical       (2 DOF)
            //UNI, // universal         (3 DOF)
            //SPH, // spherical         (3 DOF)
            FLT, // floating          (6 DOF)
            NONE};


            // DirType is designed to increase efficiency of certain multibody
            // dynamics routines.  The type field is I, J, or K, if and only if
            // the Vect3 'dir' is the corresponding unit vector.  I_, J_, and
            // K_ correspond to the negative unit vectors.  If the type is GEN,
            // then dir is a general unit vector.  Operations such as w x (w x
            // r) are greatly sped up when dirType is not GEN.

            enum  DirType {X, Y, Z, X_, Y_, Z_, GEN};



            /////////////////
            // public methods
            /////////////////


            // information about the link

            // mass of the link (kilograms)
            Real         getMass() const           { return _mass; }
            void         setMass(Real newMass)     { _mass = newMass; }

            // moments of inertia (diagonal elements of the mass matrix)
            const Vect3& getInertiaMoments() const { return _J; }
            void         setInertiaMoments(const Vect3& moments) { _J = moments; }

            // the index of this link in the multibody collection of links
            int          getIndex() const          { return _idx; }

            // returns true iff this is the base link
            bool         isBase() const            { return !inboard; }

            // Local and Global Poses (translation + rotation) of this link.
            // Provided as convenience to application; forward dynamics algo
            // does not use them.
            // (NOTE: App must call MultiBody::updatePoses() to validate
            //  these fields!)

            // the local pose (relative to the inboard joint) of this link
            const Se3&   getLocalPose() const      { return _localPose; }

            // The global pose of this link.
            const Se3&   getGlobalPose() const     { return _globalPose; }

            // set the initial position and velocity of the link (assuming
            // that this link is a 1DOF joint).  If the joint is revolute,
            // the angles should be specified in degrees.
            void set1DOFJoint(Real* initialPosition, Real* initialVelocity);

            // set the inital position and orientation of the floating link,
            // as well as the initial linear and angular velocities.  
            void setFltJoint(const Se3* initalTransformation,
                const Vect3* initialLinearVelocity,
                const Vect3* initialAngularVelocity);


            // springs/dampers/computed _torque/explicit _torque for 1 d.o.f. joints

            // Ks = spring stiffness, if spring is present 
            // _Ls = natural length of spring, if spring is present 
            void setSpring(Real Ks_, Real Ls_)
            { _Ks = (_jointType != REV ? Ks_ : Ks_ );
            _Ls = (_jointType != REV ? Ls_ : Ls_ * DEG_TO_RAD); }

            // _Kd = viscous damping coefficient, if damping is present 
            void setDamping(Real Kd_)
            { _Kd = (_jointType != REV ? Kd_ : Kd_ ); }

            // explicit applied external _torque to this joint
            void setExternalTorque(Real _torque)          { _tau = _torque; }

            // set flag to 'true' if doing computed _torque control of this joint 
            void setComputedTorqueFlag(bool flag)        { _computedTorque = flag; }

            // desired acceleration of this link if we're doing computed _torque control 
            void setDesiredAccel  (Real desiredAccel)      { _qddDes = desiredAccel; }
            void increaseDesiredQ (Real q_add)             { _qAdd = q_add; _bCompQ = true; }
            void setCopmQ         (bool bCopmQ)            { _bCompQ = bCopmQ; }
            void setDesiredPos    (Real pos);              

            void setSpringOffset  (Real springOffset)      { _addSpringOffset = springOffset; }

            LinkState getState(void);  

    private:

        //////////////////
        // private methods
        //////////////////

        Link();

        // support routines for forward dynamics

        void compXf(const Real state[]);
        void compAbsoluteVels(const Real state[], 
            const Vect3 &inboardLinkV, const Vect3 &inboardLinkW,
            Vect3 &linkV, Vect3 &linkW, int compCor);
        void compSpatialQuants(SpatialMat &IssI);
        Real jointComponent(const SpatialVect &P);
        void spatJointAcc(Real qdd);

        // forward dynamics

        void featherstonePass1(const Real sv[], const MatX &inboardXbi,
            const Vect3 &inboardLinkV, const Vect3 &inboardLinkW,
            SpatialMat &Iprop, SpatialVect &Pprop);
        void featherstonePass2(const Real sv[], Real dv[], 
            const SpatialVect &inboardAcc);

        // other

        void updatePoses();


        //////////////////
        // private data
        //////////////////

        MultiBody *mb;  // pointer to the MultiBody this link belongs to

        int _idx;       // the idx of this Link in mb's vector of Links

        Link *inboard;  // the unique link inboard to this one (parent in the tree)

        std::vector<Link *> _outboards; // list of outboard (children) links

        // Index of this Link's state in the multibody state vector.  If the
        // inboard joint has 1 d.o.f., and sv is the MultiBody state vector,
        // then sv[svIdx] is the joint position and sv[svIdx+1] is the joint
        // velocity.  If the inboard joint is floating (which is only
        // allowed for the base link), then sv[svIdx]..sv[svIdx+12] is the
        // RBConfig of this Link.
        int _svIdx;

        // poses in the local and global frames
        Se3 _globalPose;  
        Se3 _localPose;

        ///////////////////////////////////////////////////////////
        // Stuff related to the (unique) inboard joint of this link
        ///////////////////////////////////////////////////////////

        JointType _jointType;  // type of joint
        DirType   _dirType;    // type (direction code) of joint dir

        Vect3 _dir;    // direction of joint in this link's coords; was dirO

        // springs/dampers/computed _torque/explicit _torque for 1 d.o.f. joints

        Real _Ks;            // spring stiffness, if spring is present 
        Real _Ls;            // natural length of spring, is spring is present 
        Real _Kd;            // vicous damping coefficient, if damping is present 
        int _computedTorque; // true iff doing computed _torque ctrl of this jnt 
        Real _qddDes;        // desired acceleration of this jnt if we're doing
        // computed _torque control 
        Real _tau;           // explicit applied external _torque to this joint
        Real _qAdd;
        bool _bCompQ;
        Real _addSpringOffset;


        ///////////////////////////////////////
        // xformations across the inboard joint
        ///////////////////////////////////////

        // xforms independent of joint angle
        MatX _Xij;         // xform: inboard link to joint coords 
        MatX _Xjo;         // xform: joint to outboard link coords 
        // pre-compiled (constant) portion of _Xf (below)
        MatX _Xf0;        // pre-modified joint xformation (fwd); was T

        // These are computed for revolute joints.  They accelerate the
        // computation of Tf from Tf0 in cases when joint axes are aligned
        // in some way with link axes.
        int halfRotCode;   // ROT-CODE(Rjo), in range 0-23 
        int fullRotCode;   // ROT-CODE(Rij)+24*ROT-CODE(Ril[i]), in range 0-575

        MatX _Xf;  // fwd joint xform:  inboard link's frame -> this link's frame


        /////////////////////////////////////////////////////////////
        // Quantities used in Featherstone forward dynamics algorithm
        /////////////////////////////////////////////////////////////

        // Let r be vector from joint to the origin of link coords.  For
        // revolute joints we compile:
        Vect3 _mom;    // dir x r: moment part of the spatial joint 
        Vect3 _mom2;   // dir x (dir x r): used in calculating _coriolis terms 

        SpatialVect _coriolis;  // spatial _coriolis term
        SpatialVect _acc;       // spatial acceleration
        SpatialVect _Is;        // Featherstone expression
        Real _torque;           // _torque on joint
        Real _sIs;              // analog of (1 / moment of inertia)
        SpatialMat I;          // articulated spatial inertia

        Real _mass; // mass of link
        Vect3 _J;   // moments of inertia (diagonal el'ts of mass matrix)

        Real  _qdd; // acceleration of inboard link

    };

} // namespace mrt

#endif  // LINK_H


