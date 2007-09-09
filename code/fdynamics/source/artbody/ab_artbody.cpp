// standard
#include <assert.h>
// this system
#include "ab_adapter.h"
#include "ab_link.h"
#include "ab_joint.h"
#include "ab_artbody.h"

namespace ab
{

ArtBody::ArtBody() :
_isInited(false),
_mb(NULL),
_isJoinedRoot(false)
{
   ;
}

ArtBody::~ArtBody()
{
  assert(_mb);
  delete _mb;
  _mb = NULL;
}

Link* ArtBody::getRoot(void)
{
  if (_nodes.size() > 0) {
    return _nodes[0].link;
  }

  return NULL;
}

bool ArtBody::setRoot(Link* root, Joint* rootJoint)
{
    if (getRoot()) {
        return false;
    }
    
    bool total_rc = true;
    // second condition means that joint isn't 6DOFs (floating) joint
    if (rootJoint && (rootJoint->_nInternalLinks < 5)) {
        _isJoinedRoot = true;
        total_rc = rootJoint->connect(NULL, root);        
    } else {
        _isJoinedRoot = false;
        total_rc = (addChild(NULL, root) > 0) ? true : false;
    }
    
    return total_rc;
}

Joint* ArtBody::createJoint(int nDOF, JointDOFParams* dofsParams)
{
    return new Joint(nDOF, dofsParams, this);
}


int ArtBody::addChild(Link* parent, Link* child)
{
  if (_isInited) {
    return -1;
  }

  // for root node
  if (NULL == parent) {
    // only ONE root node allowed
    if (_nodes.size() > 0) {
      return -1;
    }

    _nodes.push_back(_Node(child));

    return 0;
  }

  unsigned int idx;
  for (idx = 0; idx < _nodes.size(); idx++) {
    if (_nodes[idx].link == parent) {
      // add new link to _nodes vector
      _Node node(child);
      node.parentId = idx;
      int new_idx = (int)_nodes.size();
      _nodes.push_back(node);         

      // add ref to new node in parent childId vector
      _nodes[idx].childId.push_back(new_idx);

      // recalculate local matrix of child
       child->setLocalMatr(parent->getLocalMatr() * child->getParams().modelMatr);
      // child->setLocalMatr(child->getParams().modelMatr * parent->getLocalMatr());

      return new_idx;
    }
  }

  // error - parent not found
  return -1;
}

Link* ArtBody::getParent(Link* node)
{
  unsigned int idx;
  if (node == getRoot()) {
    return NULL;
  }

  for (idx = 0; idx < _nodes.size(); idx++) {
    if (_nodes[idx].link == node) {
      return _nodes[_nodes[idx].parentId].link;
    }
  }

  // error - not found
  return NULL;
}

int ArtBody::getParent(int index)
{
  if (index < 0 || index >= (int)_nodes.size()) {
    return -1;
  }

  return _nodes[index].parentId;
}

Link* ArtBody::getLink(int index)
{
  if (index < 0 || index >= (int)_nodes.size()) {
    return NULL;
  }

  return _nodes[index].link;
}

bool ArtBody::init(void)
{
    if (_isInited) {
        return true;
    }

    // init Mirtich MultiBody
    int numLinks = (int)_nodes.size();
    _mb = new mrt::MultiBody(numLinks);   

    for (int i = 0; i < numLinks; i++) {
        mrt::Link& mrt_link = _mb->getLink(i);
        ab::Link* obj_link = getLink(i);

        mrt::Link::JointType jointType = mrt::Link::REV;
        // for base link - floating (6 dof) joint
        if ((i == 0) && (_isJoinedRoot == false)) {
            jointType = mrt::Link::FLT;
        }

        int parentIndex = _nodes[i].parentId;
        ab::Link *parent = getLink(parentIndex);
        mrt::Link* parentLink = NULL;
        if (parent) {
            parentLink = &_mb->getLink(parentIndex);         
        }
        const LinkParams& params = obj_link->getParams();        
        const ab::LinkPhysParams ph_params = params.toPhysParams();

        mrt_link.setMass(ph_params.mass); 
        mrt::Vect3 inertia;
        ab::Adapter::Vect4ToMirt(ph_params.inertia, inertia);
        mrt_link.setInertiaMoments(inertia);  

        // set up hierarchy
        ab::Vect4 jointLoc, jointAxis, jointRef, refVector;

        refVector = params.jointAxis.cross(params.jointAxis.getPerpVector());
        refVector = params.jointAxis.cross(refVector);

        jointLoc  =  params.modelMatr.getTranslate();
        jointAxis = params.modelMatr.transformAsVector(params.jointAxis);
        jointRef  = params.modelMatr.transformAsVector(refVector);
        mrt::Vect3 inLoc, inAxis, inRef, outLoc, outAxis, outRef;
        ab::Adapter::Vect4ToMirt(jointAxis, inAxis);
        ab::Adapter::Vect4ToMirt(jointRef, inRef);
        if (parent) {
            ab::Adapter::Vect4ToMirt(ab::Link::VectLocalToMirtCoord(parent, jointLoc), inLoc);
        } else {
            ab::Adapter::Vect4ToMirt(ab::Link::VectLocalToMirtCoord(obj_link, jointLoc), inLoc);
        }        
        ab::Adapter::Vect4ToMirt(params.jointAxis, outAxis);
        ab::Adapter::Vect4ToMirt(refVector, outRef);
        ab::Adapter::Vect4ToMirt(ab::Link::VectLocalToMirtCoord(obj_link, ab::Vect4::Zero), outLoc);

        _mb->connect(jointType, 
            parentLink, 
            inLoc, inAxis, inRef,
            &mrt_link, 
            outLoc, outAxis, outRef);

        // set initial joint positions / velocities / springs / dampers
        if (i == 0 && jointType == mrt::Link::FLT) {
            mrt::Vect3 lin_vel, ang_vel;
            ab::Adapter::Vect4ToMirt(params.base_lin_vel, lin_vel);
            ab::Adapter::Vect4ToMirt(params.base_ang_vel, ang_vel);
            mrt_link.setFltJoint(NULL, &lin_vel, &ang_vel);            
        } else {
            mrt::Real vel = ab::RadToDeg(params.generalized_coord_p);
            mrt_link.set1DOFJoint(NULL, &vel);            
            mrt_link.setSpring(params.springK, 0);
            mrt_link.setDamping(params.dumpK);
        }
    }

    _isInited = true;

    return true;
}

void ArtBody::applyMoment(int linkIndex, Real moment)
{
  if (!_isInited) {
    return;
  }

  if (getLink(linkIndex) != NULL) {
    mrt::Link& mrt_link = _mb->getLink(linkIndex);
    mrt_link.setExternalTorque(moment);
  }

  return;
}

void ArtBody::applyMoment(Link* node, Real moment)
{
    if (!_isInited) {
        return;
    }

    int linkId = _findLink(node);
    if (linkId != -1) {
        applyMoment(linkId, moment);
    }

    return;
}


void ArtBody::setCompAccel(int linkIndex, Real acc)
{
  if (!_isInited) {
    return;
  }

  if (getLink(linkIndex) != NULL)  {
    mrt::Link& mrt_link = _mb->getLink(linkIndex);    
    mrt::LinkState st = mrt_link.getState();
    mrt_link.setDesiredAccel(st.acc + acc);
    mrt_link.setComputedTorqueFlag(true);
  }

  return;
}

void ArtBody::setDesiredPos(Link* node, Real pos)
{
    if (!_isInited) {
        return;
    }

    int linkId = _findLink(node);

    if (getLink(linkId) != NULL)  {
        mrt::Link& mrt_link = _mb->getLink(linkId);    
        mrt::LinkState st = mrt_link.getState();
        mrt_link.setDesiredPos(st.pos + pos);
    }

}

void ArtBody::setSpringOffset(Link* node, Real springOffset)
{
    if (!_isInited) {
        return;
    }

    int linkId = _findLink(node);

    if (getLink(linkId) != NULL)  {
        mrt::Link& mrt_link = _mb->getLink(linkId);    
        mrt_link.setSpringOffset(springOffset);
    }

}


void ArtBody::processTimer(Real elapsed)
{
  if (!_isInited) {
    return;
  }
  
  _mb->evolve(0, elapsed);
  _mb->updatePoses();

  _CopyPosesFromArticulatedBody();
}

//
// Private
//
int ArtBody::_findLink(Link* link)
{
    int numLinks = (int)_nodes.size();
    for (int i = 0; i < numLinks; i++) {
        _Node& curNode = _nodes[i];
        if (curNode.link == link) {
            return i;
        }
    }

    return -1;
}


void ArtBody::_updateNodesMatr(void)
{
  return;   
}

void ArtBody::_CopyPosesFromArticulatedBody(void)
{
  static bool bFirstFrame = true;

  int numLinks = (int)_nodes.size();
  for (int i = 0; i < numLinks; i++) {
    mrt::Link& curMrtLink = _mb->getLink(i);
    mrt::Se3 globalSe3 = curMrtLink.getGlobalPose();

    Matr4 localMatr;
    Adapter::Se3ToMatr4(globalSe3, localMatr);

    Link* link = getLink(i);
    Link* parent = getParent(link);

    Matr4 parentLocal;

    if (parent) {
      parentLocal = parent->getLocalMatr();
    }

    Matr4 invParentLocal(parentLocal);
    invParentLocal.invert();

    Matr4 modelMatr = invParentLocal * localMatr;
    modelMatr = Link::MatrLocalToMirtCoord(link, modelMatr);

    localMatr = parentLocal * modelMatr;
    link->setLocalMatr(localMatr);

    mrt::LinkState mrtState = curMrtLink.getState();
    link->updateState(mrtState.pos, mrtState.vel, mrtState.acc);

    Real potE = 0.0, kinE = 0.0;
    _compLinkEnergy(i, potE, kinE);
    //        link->updateEnergy(potE, kinE, bFirstFrame);

    curMrtLink.setExternalTorque(0.0);
    curMrtLink.setDesiredAccel(0.0);
    curMrtLink.setComputedTorqueFlag(false);
  }    
  bFirstFrame = false;
}

void ArtBody::_compLinkEnergy(int linkIndex, Real& potE, Real& kinE)
{
  mrt::Link& mrtLink = _mb->getLink(linkIndex);
  mrt::LinkState mrtState = mrtLink.getState();

  Link* link = getLink(linkIndex);

  //    potE = link->getParams().mass * _mb->getGravity().y * ;
}

} // end of namespace ab