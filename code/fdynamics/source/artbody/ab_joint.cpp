//////////////////////////////////////////////////////////////////////////
// joint.cpp 
//////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <gl/glut.h>
#include <assert.h>

#include "ab_types.h"
#include "ab_adapter.h"
#include "ab_vect4.h"
#include "ab_matr4.h"
#include "ab_link.h"
#include "ab_joint.h"
#include "ab_artbody.h"


using namespace ab;

Joint::Joint(int nDOF, JointDOFParams* dofsParams, ArtBody* pOwner) :
_nDOF(nDOF),
_pOwner(pOwner)
{
    _nInternalLinks = _nDOF - 1; // because 1 dof explicit by links connect
    _explicitJoint = dofsParams[0];
    memset(_links, 0, sizeof(_links));
    if (_nInternalLinks > 0) {
        // create internal links for additional DOFs
        LinkParams lp;
        lp.lenght    = 0.0;
        lp.mass      = 0.0;
        lp.inertia = ab::Vect4(1.0, 1.0, 1.0) * (2.0 / 5.0 * lp.mass);
        lp.centerMassPos = ab::Vect4::Zero;        
        for (int i = 0; i < _nInternalLinks; i++) {
            JointDOFParams& curDOFParams = dofsParams[i + 1];
            lp.modelMatr = ab::Matr4::Identity;
            lp.jointAxis = curDOFParams.axis;
            lp.springK   = curDOFParams.springK;
            lp.dumpK     = curDOFParams.dumpK;
            _links[i] = new Link(lp);
        }
    }    
}




bool Joint::connect(Link* inboard, Link* outboard)
{
    if (_pOwner->isInited()) {
        return false;
    }

    // initial correctness test
    if (inboard == NULL) {
        // try to add root joint
        if (_pOwner->getRoot()) {
            // art_body already has a root
            return false;
        }
    } else {
        _inLinkIdx = _pOwner->_findLink(inboard);
        if (_inLinkIdx < 0) {
            // inboard link isn't connected to MB yet
            return false;
        }
    }    

    // check 'floating root' case
    if (inboard == NULL && _nInternalLinks == 5) {
        return (_pOwner->addChild(NULL, outboard) == 0) ? true : false;
    }
    
    if (_pOwner->_findLink(outboard) > -1) {
        // outboard link is connected to MB -> cycle
        return false;
    }

    if (_nInternalLinks > 0) {
        _links[0]->setModelMatr(outboard->getModelMatr());
        outboard->setModelMatr(ab::Matr4::Identity);
    }

    Link* parent = inboard;
    // add internal links
    for (int i = 0; i < _nInternalLinks; i++) {
        _pOwner->addChild(parent, _links[i]);
        parent = _links[i];
    }

    //add outboard link
    outboard->_params.jointAxis = _explicitJoint.axis;
    outboard->_params.springK   = _explicitJoint.springK;
    outboard->_params.dumpK     = _explicitJoint.dumpK;
    _outLinkIdx = _pOwner->addChild(parent, outboard);
    if (_outLinkIdx == -1) {
        return false;
    }
    
    return true;
}

Joint::~Joint()
{
    for (int i = 0; i < _nInternalLinks; i++) {
        delete _links[i];
    }
}



