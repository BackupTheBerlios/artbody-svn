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
#include "Link.h"
#include "MultiBody.h"

using namespace mrt;

#define _IsZero(value) ( abs(value) < 0.0001 )


///////////////////////////////////////////////////////////////////////////////
//  class Link
///////////////////////////////////////////////////////////////////////////////

Link::Link()
{
    inboard = NULL;
    _svIdx   = -1;

    _jointType = NONE;

    _computedTorque = 0;
    _Ks = _Ls = _Kd = _tau = _qddDes = 0.0;

    _bCompQ = false;
    _qAdd   = 0.0;

    _mass = 1.0;
    _J = Vect3(1.0, 1.0, 1.0);
    _addSpringOffset = 0.f;
}


void Link::set1DOFJoint(Real* pos, Real* vel)
{
    if (_svIdx == -1) {
        crash("Link::setJoint(): Multibody not initialized");
    }

    if (pos == NULL && vel == NULL) {
        return;
    }

    if (_jointType == PRI) {
        if (pos) {
            mb->getState(_svIdx)     = *pos;
        }        
        if (vel) {
            mb->getState(_svIdx + 1) = *vel;
        }        
    }
    else if (_jointType == REV) {
        if (pos) {
            mb->getState(_svIdx)     = (*pos) * DEG_TO_RAD;
        }
        if (vel) {
            mb->getState(_svIdx + 1) = (*vel) * DEG_TO_RAD;
        }
    }
    else crash("Link::setJoint():  invalid joint type");
}


void Link::setFltJoint(const Se3* Tbi, const Vect3* v, const Vect3* w)
{
    if (_svIdx == -1) {
        crash("Link::setPose(): Multibody not initialized");
    }

    if (_jointType != FLT) {
        crash("Link::setPose(): joint type not FLT");
    }

    if (Tbi) {
        ((RBConfig *) &mb->getState(_svIdx))->Tbi = *Tbi;
    }
    if (v) {    
        ((RBConfig *) &mb->getState(_svIdx))->v   = *v;
    }
    if (w) {
        ((RBConfig *) &mb->getState(_svIdx))->w   = *w;
    }

}




//  Efficiently computing maps across joints  //////////////////////////////////

/* 

Let R be a rotation matrix mapping frame F to frame G, where F and
G are both right-handed frames, and such that the basis vectors in
one system are each aligned with a (+ve or -ve) basis vector in the
other system.

Define "dir" as the direction of F's z-axis in G's coords; define
"ref" as the direction of F's x-axis in G's coords.  (These
correspond to the dir and ref axes of a joint frame.)  Then there
are 24 possible rotations between F and G, numbered 0 through 23.
Each matrix R in the char below rotates vectors in F to vectors in
G.


\  ref
\       +i           -i           +j           -j           +k           -k
d \ __________________________________________________________________________
i  |
r  |                          |      +1|   |      +1|   |      +1|   |      +1|
+i|                          |+1      |   |-1      |   |   -1   |   |   +1   |
|                          |   +1   |   |   -1   |   |+1      |   |-1      |
|
|                              00           01           02           03
|
|
|                          |      -1|   |      -1|   |      -1|   |      -1|
-i|                          |+1      |   |-1      |   |   +1   |   |   -1   |
|                          |   -1   |   |   +1   |   |+1      |   |-1      |
|
|                              04           05            06          07
|
|
||+1      |   |-1      |                             |   +1   |   |   -1   |
+j||      +1|   |      +1|                             |      +1|   |      +1|
||   -1   |   |   +1   |                             |+1      |   |-1      |
|
|    10           11                                     08           09
|
|
||+1      |   |-1      |                             |   -1   |   |   +1   |
-j||      -1|   |      -1|                             |      -1|   |      -1|
||   +1   |   |   -1   |                             |+1      |   |-1      |
|
|    14           15                                     12           13
|
|
||+1      |   |-1      |   |   -1   |   |   +1   |
+k||   +1   |   |   -1   |   |+1      |   |-1      |
||      +1|   |      +1|   |      +1|   |      +1|
|
|    16           17           18           19
|
|
||+1      |   |-1      |   |   +1   |   |   -1   |
-k||   -1   |   |   +1   |   |+1      |   |-1      |
||      -1|   |      -1|   |      -1|   |      -1|
|
|   20            21           22           23



Now imagine 3 coordinate frames connected via 3 transformations:

inboard     X1=(R1;d1)      Xj=(Rj;dj)      X2=(R2;d2)     outboard
link     ------------->    --------->    ------------->     link
frame                      joint frame                      frame


X, the total transformation from inboard to outboard link frame, is
given by the composition (X2)(Xj)(X1).  In the case of a prismatic
joint, Rj is the identity and dj is (0,0,-q)^T, where q is the joint
angle.  Multiplying the matrices together, we find:

| R2 R1 | R2 d1 + d2 |     | -v |
X = | -------------------| + q |----|
|   0   |  1         |     | 0  |

where v is the third column of R2.  The matrix on the left is
constant and can be pre-compiled.  Only the vector on the right is
dependent on q.  In the case of a revolute joint, dj = 0, and Rj is
a rotation by angle -q about the z axis.  Multiplying matrices, we
find:

| R2 Rj R1 | d1z v + d2 + R2 h |
X = | -----------------------------|
|     0    |          1        |

where d1z is the z-component of d1, v is the third column of R2, and
h is the vector (d1x cos q + d1y sin q, - d1x sin q + d1y cos q, 0)^T.
The terms (d1z v + d2) are fixed and can be pre-compiled.
*/

////////////////////////////////////////////////////////////////////////////////
//  MultiBody forward dynamics
////////////////////////////////////////////////////////////////////////////////


//  Compute the _Xf matrix of a prismatic or revolute joint from the
//  _Xf0 matrix.  This is the forward xformation across the joint, from
//  the inboard link's coords to the outboard link's coords.

void Link::compXf(const Real sv[]) // was jointMat
{
    Real s, c, q, dx, dy;
    Vect3 v;
    Mat3 R;
    Mat3 Rj = Mat3::ID;

    q = sv[_svIdx];
    _Xf = _Xf0;

    if (_jointType == PRI) {
        //        switch (_dirType) {
        //            case X : 
        //                _Xf.trans().x -= q; 
        //                break;
        //            case X_: 
        //                _Xf.trans().x += q; 
        //                break;
        //            case Y : 
        //                _Xf.trans().y -= q; 
        //                break;
        //            case Y_: 
        //                _Xf.trans().y += q; 
        //                break;
        //            case Z : 
        //                _Xf.trans().z -= q; 
        //                break;
        //            case Z_: 
        //                _Xf.trans().z += q; 
        //                break;
        //            case GEN:
        //                v = _Xjo.rot().zcol();
        //                _Xf.trans().displace(v, -q);
        //                break;
        //        }
        v = _Xjo.rot().zcol();
        _Xf.trans().displace(v, -q);
    } else if (_jointType == REV) {
        s = sin(q);
        c = cos(q);
        // joint frames not axes aligned with both link frames - do
        // general computation
        Rj.xrow().x = c;
        Rj.yrow().y = c;
        Rj.xrow().y = +s;
        Rj.yrow().x = -s;
        R.mult(Rj, _Xij.rot());
        Rj.xform(_Xij.trans(), v);
        _Xf.set(R, v);
        _Xf.premult(_Xjo);    
    }

    return;
}


//  Compute linkV and linkW given the current joint positions and
//  velocities. linkV (resp. linkW) is the absolute linear
//  (resp. angular) velocity (i.e. relative to the inertial frame),
//  but it's expressed in the link frame. If compCor is true, compute
//  the Coriolis vector as well.  This routine assumes the link
//  transform _Xf is current.
void Link::compAbsoluteVels(const Real sv[], // state vector
                            const Vect3 &inboardLinkV, const Vect3 &inboardLinkW,
                            Vect3 &linkV, Vect3 &linkW, int compCor) 
{
    Real qd;             // joint velocity
    Vect3 w;             // w for inboard link, xformed to this link's frame
    Vect3 r;
    Vect3 wxr;           // w cross r
    Vect3 v, v2;         // subexpressions & temp storage 

    qd = sv[_svIdx + 1]; // at least for 1 dof joints...

    if (isBase()) { 
        switch(_jointType) {
        case PRI:
            linkW = Vect3::ZERO;
            linkV.scale(_dir, qd);
            if (!compCor) break;
            _coriolis = SpatialVect::ZERO;
            break;
        case REV:
            linkW.scale(_dir, qd);
            linkV.scale(_mom, qd);
            if (!compCor) break;
            _coriolis.top() = Vect3::ZERO;
            _coriolis.bot().scale(_mom2, qd * qd);
            break;
        case FLT:
            {   
                RBConfig * rbBaseConf = (RBConfig *)(&sv[_svIdx]);
                linkW = rbBaseConf->w;
                rbBaseConf->Tbi.invXformVect(rbBaseConf->v, linkV);
                // don't compute _coriolis if joint 0 is floating since it's not used
            }

            break;
        }
    }

    else { // not base link

        // r = vector from inbrd link origin to outbrd (this) link origin, in
        // outbrd (this) link's coords
        r.negate(_Xf.trans());

        _Xf.xformVect(inboardLinkW, linkW);
        w = linkW;
        _Xf.xformVect(inboardLinkV, linkV);
        wxr.cross(w, r);
        linkV.add(wxr);

        switch(_jointType) {
            case PRI:
                linkV.displace(_dir, qd);
                if (!compCor) break;
                _coriolis.top() = Vect3::ZERO;
                //                switch(_dirType) {
                //                    case X:  wxr.x += 2 * qd; break;
                //                    case X_: wxr.x -= 2 * qd; break;
                //                    case Y:  wxr.y += 2 * qd; break;
                //                    case Y_: wxr.y -= 2 * qd; break;
                //                    case Z:  wxr.z += 2 * qd; break;
                //                    case Z_: wxr.z -= 2 * qd; break;
                //                    case GEN: wxr.displace(_dir, 2 * qd); break;
                //                }
                wxr.displace(_dir, 2 * qd); break;
                _coriolis.bot().cross(w, wxr);
                break;

            case REV:
                linkW.displace(_dir, qd);
                linkV.displace(_mom, qd);
                if ( compCor ) {      
                    //                    switch ( _dirType ) {
                    //                        case X:  _coriolis.top().set(      0,  w.z*qd, -w.y*qd); break;
                    //                        case X_: _coriolis.top().set(      0, -w.z*qd,  w.y*qd); break;
                    //                        case Y:  _coriolis.top().set(-w.z*qd,       0,  w.x*qd); break;
                    //                        case Y_: _coriolis.top().set( w.z*qd,       0, -w.x*qd); break;
                    //                        case Z:  _coriolis.top().set( w.y*qd, -w.x*qd,       0); break;
                    //                        case Z_: _coriolis.top().set(-w.y*qd,  w.x*qd,       0); break;
                    //                        case GEN: 
                    //                            v.cross(w, _dir);
                    //                            _coriolis.top().scale(v, qd);
                    //                            break;
                    //                    }
                    {
                        // top 
                        Vect3 v_(_dir);
                        v_.scale(qd);
                        v.cross(w, v_);                        
                        _coriolis.top().set(v.x, v.y, v.z);

                        // bottom
                        Vect3 vvd(_mom2);
                        vvd.scale(qd * qd);
                        Vect3 vd(_mom);
                        vd.scale(qd);
                        Vect3 wwr;
                        wwr.cross(w, wxr);
                        Vect3 _2wvd;
                        _2wvd.cross(w, vd);
                        _2wvd.scale(2);

                        _coriolis.bot().set(0, 0, 0);
                        _coriolis.bot().add(wwr);
                        _coriolis.bot().add(_2wvd);
                        _coriolis.bot().add(vvd);
                    }

                }
                break;
        }
    }
}



// Compute spatial quantities needed for the featherstone algo.
// Assumes the Link's articulated body inertia I is already computed
// (N.B(). this is spatially symmetric).  The function computes _Is, _sIs,
// and IssI, where s is the spatial vector for the inboard joint to
// the link.  _Is and _sIs are members of class link; IssI is returned
// via the parameter.

void Link::compSpatialQuants(SpatialMat &IssI)
{
    Vect3 v;  // tmp 
    Real  divisor;

    switch (_jointType) {
    case PRI:
        //        switch(_dirType) {
        //            case X: 
        //                _Is.top() = I.B().xcol();
        //                _Is.bot() = I.D().xcol();
        //                divisor = I.B().xrow().x;        
        //                break;
        //            case X_:
        //                _Is.top().negate(I.B().xcol());
        //                _Is.bot().negate(I.D().xcol());
        //                divisor = I.B().xrow().x;        
        //                break;
        //            case Y: 
        //                _Is.top() = I.B().ycol();
        //                _Is.bot() = I.D().ycol();
        //                divisor = I.B().yrow().y;        
        //                break;
        //            case Y_:
        //                _Is.top().negate(I.B().ycol());
        //                _Is.bot().negate(I.D().ycol());
        //                divisor = I.B().yrow().y;
        //                break;
        //            case Z: 
        //                _Is.top() = I.B().zcol();
        //                _Is.bot() = I.D().zcol();
        //                divisor = I.B().zrow().z;
        //                break;
        //            case Z_:
        //                _Is.top().negate(I.B().zcol());
        //                _Is.bot().negate(I.D().zcol());
        //                divisor = I.B().zrow().z;
        //                break;
        //            case GEN:
        //                I.B().xform(_dir, _Is.top());
        //                I.D().xform(_dir, _Is.bot());
        //                divisor = _dir.dot(_Is.top());
        //                break;
        //        }
        I.B().xform(_dir, _Is.top());
        I.D().xform(_dir, _Is.bot());
        divisor = _dir.dot(_Is.top());
        break;        
    case REV:
        //        switch(_dirType) {
        //            case X:
        //                _Is.top().set( I.A().xrow().x + _mom.y*I.B().xrow().y + _mom.z*I.B().xrow().z,
        //                    I.A().yrow().x + _mom.y*I.B().yrow().y + _mom.z*I.B().yrow().z,
        //                    I.A().zrow().x + _mom.y*I.B().zrow().y + _mom.z*I.B().zrow().z);
        //                _Is.bot().set( I.C().xrow().x + _mom.y*I.D().xrow().y + _mom.z*I.D().xrow().z,
        //                    I.C().yrow().x + _mom.y*I.D().yrow().y + _mom.z*I.D().yrow().z,
        //                    I.C().zrow().x + _mom.y*I.D().zrow().y + _mom.z*I.D().zrow().z);
        //                divisor = 1 / (_Is.bot().x + _mom.y * _Is.top().y + _mom.z * _Is.top().z);
        //                break;
        //            case X_:
        //                _Is.top().set(-I.A().xrow().x + _mom.y*I.B().xrow().y + _mom.z*I.B().xrow().z,
        //                    -I.A().yrow().x + _mom.y*I.B().yrow().y + _mom.z*I.B().yrow().z,
        //                    -I.A().zrow().x + _mom.y*I.B().zrow().y + _mom.z*I.B().zrow().z);
        //                _Is.bot().set(-I.C().xrow().x + _mom.y*I.D().xrow().y + _mom.z*I.D().xrow().z,
        //                    -I.C().yrow().x + _mom.y*I.D().yrow().y + _mom.z*I.D().yrow().z,
        //                    -I.C().zrow().x + _mom.y*I.D().zrow().y + _mom.z*I.D().zrow().z);
        //                divisor = (-_Is.bot().x + _mom.y * _Is.top().y + _mom.z * _Is.top().z);
        //                break;
        //            case Y:
        //                _Is.top().set( I.A().xrow().y + _mom.z*I.B().xrow().z + _mom.x*I.B().xrow().x,
        //                    I.A().yrow().y + _mom.z*I.B().yrow().z + _mom.x*I.B().yrow().x,
        //                    I.A().zrow().y + _mom.z*I.B().zrow().z + _mom.x*I.B().zrow().x);
        //                _Is.bot().set( I.C().xrow().y + _mom.z*I.D().xrow().z + _mom.x*I.D().xrow().x,
        //                    I.C().yrow().y + _mom.z*I.D().yrow().z + _mom.x*I.D().yrow().x,
        //                    I.C().zrow().y + _mom.z*I.D().zrow().z + _mom.x*I.D().zrow().x);
        //                divisor = (_Is.bot().y + _mom.z * _Is.top().z + _mom.x * _Is.top().x);
        //                break;
        //            case Y_:
        //                _Is.top().set(-I.A().xrow().y + _mom.z*I.B().xrow().z + _mom.x*I.B().xrow().x,
        //                    -I.A().yrow().y + _mom.z*I.B().yrow().z + _mom.x*I.B().yrow().x,
        //                    -I.A().zrow().y + _mom.z*I.B().zrow().z + _mom.x*I.B().zrow().x);
        //                _Is.bot().set(-I.C().xrow().y + _mom.z*I.D().xrow().z + _mom.x*I.D().xrow().x,
        //                    -I.C().yrow().y + _mom.z*I.D().yrow().z + _mom.x*I.D().yrow().x,
        //                    -I.C().zrow().y + _mom.z*I.D().zrow().z + _mom.x*I.D().zrow().x);
        //                divisor = (-_Is.bot().y + _mom.z * _Is.top().z + _mom.x * _Is.top().x);
        //                break;
        //            case Z:
        //                _Is.top().set( I.A().xrow().z + _mom.x*I.B().xrow().x + _mom.y*I.B().xrow().y,
        //                    I.A().yrow().z + _mom.x*I.B().yrow().x + _mom.y*I.B().yrow().y,
        //                    I.A().zrow().z + _mom.x*I.B().zrow().x + _mom.y*I.B().zrow().y);
        //                _Is.bot().set( I.C().xrow().z + _mom.x*I.D().xrow().x + _mom.y*I.D().xrow().y,
        //                    I.C().yrow().z + _mom.x*I.D().yrow().x + _mom.y*I.D().yrow().y,
        //                    I.C().zrow().z + _mom.x*I.D().zrow().x + _mom.y*I.D().zrow().y);
        //                divisor = (_Is.bot().z + _mom.x * _Is.top().x + _mom.y * _Is.top().y);
        //                break;
        //            case Z_:
        //                _Is.top().set(-I.A().xrow().z + _mom.x*I.B().xrow().x + _mom.y*I.B().xrow().y,
        //                    -I.A().yrow().z + _mom.x*I.B().yrow().x + _mom.y*I.B().yrow().y,
        //                    -I.A().zrow().z + _mom.x*I.B().zrow().x + _mom.y*I.B().zrow().y);
        //                _Is.bot().set(-I.C().xrow().z + _mom.x*I.D().xrow().x + _mom.y*I.D().xrow().y,
        //                    -I.C().yrow().z + _mom.x*I.D().yrow().x + _mom.y*I.D().yrow().y,
        //                    -I.C().zrow().z + _mom.x*I.D().zrow().x + _mom.y*I.D().zrow().y);
        //                divisor = (-_Is.bot().z + _mom.x * _Is.top().x + _mom.y * _Is.top().y);
        //                break;
        //            case GEN:
        //                I.A().xform(_dir, _Is.top());
        //                I.B().xform(_mom, v);
        //                _Is.top().add(v);
        //                I.C().xform(_dir, _Is.bot());
        //                I.D().xform(_mom, v);
        //                _Is.bot().add(v);
        //                divisor = (_mom.dot(_Is.top()) + _dir.dot(_Is.bot()));
        //                break;
        //        }

        //        I.A().xform(_dir, _Is.top());
        //        I.B().xform(_mom, v);
        //        _Is.top().add(v);
        //        I.C().xform(_dir, _Is.bot());
        //        I.D().xform(_mom, v);
        //        _Is.bot().add(v);
        //        divisor = (_mom.dot(_Is.top()) + _dir.dot(_Is.bot()));

        {
            SpatialVect s(_dir, _mom);
            I.xform(s, _Is);
            divisor = s.dot(_Is);
        }
        break;
    }



    _sIs = !_IsZero(divisor) ? (1.0 / divisor) : 0.0;

    // Calculate the quantity Iss'I from the quantity _Is, where ' denotes
    // spatial transpose.  This can also be written (_Is)(_Is)' since I is
    // spatially symmetric.  Note that we use the fact that the product is
    // also a spatially symmetric matrix to reduce computation.

    // compute A component
    IssI.A().xrow().set(_Is.top().x * _Is.bot().x,
        _Is.top().x * _Is.bot().y,
        _Is.top().x * _Is.bot().z);
    IssI.A().yrow().set(_Is.top().y * _Is.bot().x,
        _Is.top().y * _Is.bot().y,
        _Is.top().y * _Is.bot().z);
    IssI.A().zrow().set(_Is.top().z * _Is.bot().x,
        _Is.top().z * _Is.bot().y,
        _Is.top().z * _Is.bot().z);

    // D component is just transpose of A component
    IssI.D().xpose(IssI.A());

    // compute B component.  This is a (standard) symmetric matrix 
    IssI.B().set(Vect3(_Is.top().x * _Is.top().x,
        _Is.top().y * _Is.top().y,
        _Is.top().z * _Is.top().z),
        Vect3(_Is.top().y * _Is.top().z,
        _Is.top().z * _Is.top().x,
        _Is.top().x * _Is.top().y));

    // compute C component.  This is a (standard) symmetric matrix
    IssI.C().set(Vect3(_Is.bot().x * _Is.bot().x,
        _Is.bot().y * _Is.bot().y,
        _Is.bot().z * _Is.bot().z),
        Vect3(_Is.bot().y * _Is.bot().z,
        _Is.bot().z * _Is.bot().x,
        _Is.bot().x * _Is.bot().y));

}


// Xform a spatial vector u in a source frame to a spatial vector v in
// a destination frame.  X is the transformation from the source frame
// to the destination frame.  u and v must be distinct!

void xformSpatVect(const MatX &X, const SpatialVect &u, SpatialVect &v)
{
    X.rot().xform(u.top(), v.top());
    X.rot().xform(u.bot(), v.bot());
    v.bot().crossAdd(X.trans(), v.top());
}

// Xform a spatial vector u in a source frame to a spatial vector v in
// a destination frame.  X is the transformation from the destination frame
// to the source frame. u and v must be distinct!

void invXformSpatVect(const MatX &X, const SpatialVect &u, SpatialVect &v)
{
    X.rot().invXform(u.top(), v.top());
    v.bot().crossAdd(u.top(), X.trans(), u.bot());
    X.rot().invXform(v.bot());
}


// Xform a spatial inertia Ia in a source frame to a spatial vector Ib in
// a destination frame.  X is the transformation from the source frame
// to the destination frame.

void xformSpatInertia(const MatX &X, const SpatialMat &Ia, SpatialMat &Ib)
{
    Mat3 A_, C_, U, v, w, y, Rt;

    Rt.xpose(X.rot());
    U.mult(X.rot(), Ia.A());
    A_.mult(U, Rt);
    U.mult(X.rot(), Ia.B());
    Ib.B().mult(U, Rt);
    U.mult(X.rot(), Ia.C());
    C_.mult(U, Rt);

    U.setSkew(X.trans());
    v.mult(Ib.B(), U);
    Ib.A().sub(A_, v);
    Ib.D().xpose(Ib.A());
    w.mult(U, A_);
    y.mult(U, v);
    Ib.C().symmetrize(w);
    Ib.C().sub(y);
    Ib.C().add(C_);
}


// Return the component (i.e. projection) of a given generalized
// spatial force P along the spatial axis of the link's inboard joint.
// Both, must be specified in the link frame.  (For the Featherstone
// algo, P is the bias force.)


Real Link::jointComponent(const SpatialVect &P)
{

    if (_jointType == PRI) {
        //        switch (_dirType) {
        //            case X:  return  P.top().x;
        //            case X_: return -P.top().x;
        //            case Y:  return  P.top().y;
        //            case Y_: return -P.top().y;
        //            case Z:  return  P.top().z;
        //            case Z_: return -P.top().z;
        //            case GEN: return _dir.dot(P.top());
        //        }
        return _dir.dot(P.top());
    } else if (_jointType == REV) {
        //        switch (_dirType) {
        //            case X:  return  P.bot().x + P.top().y * _mom.y + P.top().z * _mom.z;
        //            case X_: return -P.bot().x + P.top().y * _mom.y + P.top().z * _mom.z;
        //            case Y:  return  P.bot().y + P.top().z * _mom.z + P.top().x * _mom.x;
        //            case Y_: return -P.bot().y + P.top().z * _mom.z + P.top().x * _mom.x;
        //            case Z:  return  P.bot().z + P.top().x * _mom.x + P.top().y * _mom.y;
        //            case Z_: return -P.bot().z + P.top().x * _mom.x + P.top().y * _mom.y;
        //            case GEN: return _dir.dot(P.bot()) + _mom.dot(P.top());
        //        }
        return _dir.dot(P.bot()) + _mom.dot(P.top());
    }

    return 0.0; // never reached, but make compiler happy
}



// Initialize a link's _acc field to the spatial acceleration induced
// soley by the inboard joint acceleration.  This is the quantity (s qdd)
// where s is the spatial joint axis, and qdd is the joint's scalar
// acceleration.

void Link::spatJointAcc(Real qdd)
{
    _acc = SpatialVect::ZERO;

    if (_jointType == PRI) {
        //        switch (_dirType) {
        //            case X:  _acc.bot().x =  qdd; break;
        //            case X_: _acc.bot().x = -qdd; break;
        //            case Y:  _acc.bot().y =  qdd; break;
        //            case Y_: _acc.bot().y = -qdd; break;
        //            case Z:  _acc.bot().z =  qdd; break;
        //            case Z_: _acc.bot().z = -qdd; break;
        //            case GEN: _acc.bot().scale(_dir, qdd); break;
        //        }
        _acc.bot().scale(_dir, qdd);
    } else if (_jointType == REV) {
        //        switch (_dirType) {
        //            case X:
        //                _acc.top().x =  qdd; 
        //                _acc.bot().y = _mom.y * qdd;
        //                _acc.bot().z = _mom.z * qdd;
        //                break;
        //            case X_:
        //                _acc.top().x = -qdd; 
        //                _acc.bot().y = _mom.y * qdd;
        //                _acc.bot().z = _mom.z * qdd;
        //                break;
        //            case Y:
        //                _acc.top().y =  qdd; 
        //                _acc.bot().z = _mom.z * qdd;
        //                _acc.bot().x = _mom.x * qdd;
        //                break;
        //            case Y_:
        //                _acc.top().y = -qdd; 
        //                _acc.bot().z = _mom.z * qdd;
        //                _acc.bot().x = _mom.x * qdd;
        //                break;
        //            case Z:
        //                _acc.top().z =  qdd; 
        //                _acc.bot().x = _mom.x * qdd;
        //                _acc.bot().y = _mom.y * qdd;
        //                break;
        //            case Z_:
        //                _acc.top().z = -qdd; 
        //                _acc.bot().x = _mom.x * qdd;
        //                _acc.bot().y = _mom.y * qdd;
        //                break;
        //            case GEN:
        //                _acc.top().scale(_dir, qdd);
        //                _acc.bot().scale(_mom, qdd);
        //                break;
        //        }
        _acc.top().scale(_dir, qdd);
        _acc.bot().scale(_mom, qdd);
    }
}



void Link::featherstonePass1(
                             // inputs:			     
                             const Real sv[], // state vector
                             const MatX &inboardXbi,
                             const Vect3 &inboardLinkV,
                             const Vect3 &inboardLinkW,
                             // outputs:
                             SpatialMat &Iprop,  // articulated body inertia, xformed to inboard's frame
                             SpatialVect &Pprop // articulated z.A(). force, xformed to inboard's frame
                             )
{
    std::vector<Link *>::iterator lpi;
    Vect3 linkV;  // absolute linear velocity (link frame)
    Vect3 linkW;  // absolute angular velocity (link frame)
    Vect3 g;
    MatX Xb;   // xform:  link -> inboard (backward across inboard joint)
    MatX Xbi;  // xform:  link -> inertial
    SpatialVect P, kidP;  // z.A(). force
    SpatialVect Ic, tmpV;
    SpatialMat kidI;   // spatial inertia
    SpatialMat IssI, Ii, tmpI;


    // compute transformations for this link
    if (isBase()) {
        if (_jointType == FLT) {
            Xbi.set(((RBConfig *) &sv[_svIdx])->Tbi);
        } else {
            compXf(sv);
            Xb.invert(_Xf);
            Xbi = Xb;
        }
    } else {
        compXf(sv);
        Xb.invert(_Xf);
        Xbi.mult(inboardXbi, Xb);
    }

    // compute absolute link velocities
    compAbsoluteVels(sv, inboardLinkV, inboardLinkW, linkV, linkW, 1);

    // intialize articulated spatial inertia to isolated spatial inertia
    I.A() = I.D() = Mat3::ZERO;
    I.B().set(Vect3(_mass, _mass, _mass), Vect3::ZERO);
    I.C().set(_J, Vect3::ZERO);

    // intialize articulated spatial z.A(). force to isolated spatial z.A(). force
    Xbi.invXformVect(mb->getGravity(), g);
    P.top().scale(g, -_mass);   // account for gravity.
    P.bot().set(linkW.z * linkW.y * (_J.z - _J.y),
        linkW.x * linkW.z * (_J.x - _J.z),
        linkW.y * linkW.x * (_J.y - _J.x));

    /////////////// TESTED

    // propagate back results from kids
    FOR_EACH(_outboards, lpi) {
        (*lpi)->featherstonePass1(sv, Xbi, linkV, linkW, kidI, kidP);
        I.add(kidI);
        P.add(kidP);
    }

    if (isBase()) {  // compute spatial acceleration _acc of link
        if (_jointType == FLT) {
            Ii.inertiaInvert(I);
            Ii.xform(P, _acc);
            _acc.negate();
        } else {
            if (_computedTorque) {
                spatJointAcc(_qddDes);
            } else {
                compSpatialQuants(IssI);
                _torque = _tau - _Ks * (sv[_svIdx] + _addSpringOffset - _Ls) - _Kd * sv[_svIdx+1]
                - _Is.dot(_coriolis) - jointComponent(P);
                spatJointAcc(_torque * _sIs);
            }
            _acc.add(_coriolis);

        }
    } else { // not base link:  compute Iprop and Pprop for link
        compSpatialQuants(IssI); // also sets _Is, _sIs
        if (_computedTorque) { 
            tmpI = I;
        } else {
            tmpI.scale(IssI, _sIs);
            tmpI.sub(I, tmpI);
        }
        xformSpatInertia(Xb, tmpI, Iprop);

        if (_computedTorque) {
            tmpV.scale(_Is, _qddDes);
        } else {
            _torque = _tau - _Ks * (sv[_svIdx] + _addSpringOffset - _Ls) - _Kd * sv[_svIdx+1]
            - _Is.dot(_coriolis) - jointComponent(P);
            tmpV.scale(_Is, _torque * _sIs);
        }
        I.xform(_coriolis, Ic);
        tmpV.add(Ic);
        tmpV.add(P);
        xformSpatVect(Xb, tmpV, Pprop);
    }
    /////////////////////////// TESTED
}

void Link::featherstonePass2(
                             const Real sv[],  // state vector
                             Real dv[],        // derivative vector
                             const SpatialVect &inboardAcc)
{
    Real *qdd;
    std::vector<Link *>::iterator lpi;
    Quat qdot;
    SpatialVect v; // tmp

    qdd = dv + _svIdx + 1;

    if (isBase()) {
        if (_jointType == FLT) {
            // quaternion derivatives computed from quaternion & angular velocity
            // position derivatives specifed by velocity state
            RBConfig * rbBaseConf    = (RBConfig *)(&sv[_svIdx]);
            RBConfig * rbVelBaseConf = (RBConfig *)(&dv[_svIdx]);
            qdot.deriv(rbBaseConf->Tbi.rot(), rbBaseConf->w);
            rbVelBaseConf->Tbi.set(qdot, rbBaseConf->v);
            // lin. and ang. velocity derivatives specified by base's spatial _acc.
            rbBaseConf->Tbi.xformVect(_acc.bot(), rbVelBaseConf->v);
            rbVelBaseConf->w = _acc.top();
        } else {
            dv[_svIdx] = sv[_svIdx + 1];
            if (_computedTorque) {
                *qdd = _qddDes;
            } else {
                *qdd = _torque * _sIs;
            }
        }
    }
    ///////////////////////////// TESTED
    else {  // not base link
        dv[_svIdx] = sv[_svIdx + 1];
        xformSpatVect(_Xf, inboardAcc, v);
        if (_computedTorque) {
            *qdd = _qddDes;
        } else {
            *qdd = (_torque - _Is.dot(v)) * _sIs;
        }

        // We could skip this code for MB leaves if we only cared about
        // forward dynamics, but things like contact analysis require the
        // _acc field of the leaves, so we do it.
        spatJointAcc(*qdd);  // computes _acc for link
        _acc.add(v);
        _acc.add(_coriolis);
    }
    _qdd = *qdd;

    FOR_EACH(_outboards, lpi) (*lpi)->featherstonePass2(sv, dv, _acc);

}


// Recursively update the _pose and _Xf transforms of this and
// all descendent links of a MultiBody, based on the State.

void Link::updatePoses()
{
    std::vector<Link *>::iterator lpi;
    MatX Xb;

    // compute pose for this link
    if (isBase()) {
        if (_jointType == FLT)
            _localPose = _globalPose = ((RBConfig *) &mb->getState(0))->Tbi;
        else {
            compXf(&mb->getState(0));
            Xb.invert(_Xf);
            _localPose.set(Xb);
            _globalPose = _localPose;
        }
    }
    else {
        compXf(&mb->getState(0));
        Xb.invert(_Xf);
        _localPose.set(Xb);
        _globalPose.mult(inboard->_globalPose, _localPose);
    }

    // recurse through hierarchy
    FOR_EACH(_outboards, lpi) (*lpi)->updatePoses(); 

}

LinkState Link::getState()
{
    LinkState curState = {0.0, 0.0, 0.0};
    if (mb) {
        curState.pos = mb->getState(_svIdx);
        curState.vel = mb->getState(_svIdx + 1);
        // curState.acc = _acc.top().z;
        curState.acc = _qdd;
    }
    return curState;
}

void Link::setDesiredPos(Real pos)
{
    mb->getState(_svIdx) = pos;
}


