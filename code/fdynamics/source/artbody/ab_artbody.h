#ifndef _AB_ARTBODY_H_
#define _AB_ARTBODY_H_

// STL
#include <vector>
// Mirtich
#include "core/MultiBody.h"
// this system
#include "ab_types.h"

namespace ab
{

// Forward declaration
class  Link;
class  Joint;
struct JointDOFParams;

class ArtBody
{
friend class Joint;
public: 
  ArtBody();
  virtual ~ArtBody();

  // new interface
  Joint* createJoint(int nDOF, JointDOFParams* dofsParams);
  bool   setRoot    (Link* root, Joint* rootJoint = NULL);
  bool   init       (void);
  bool   isInited   (void) const { return _isInited; }
  Link*  getRoot   (void);

  // old interface
  int    addChild (Link* parent, Link* child);
  Link*  getParent(Link* node);
  int    getParent(int index);
  Link*  getLink  (int index);

  unsigned int getNLinks(void) const { return (int)_nodes.size(); }

  void processTimer (Real elapsed);

  void applyMoment    (int linkIndex, Real moment);
  void applyMoment    (Link* node, Real moment);
  void setCompAccel   (int linkIndex, Real acc);
  void setDesiredPos  (Link* node, Real pos);
  void setSpringOffset(Link* node, Real springOffset);

  mrt::MultiBody* getMultiBody() const { return _mb; }
//protected:
  void _updateNodesMatr             (void);
  void _CopyPosesFromArticulatedBody(void);
  void _compLinkEnergy              (int linkIndex, Real& potE, Real& kinE);
  int  _findLink                    (Link* link);


  struct _Node {
    _Node(Link* link_) : 
    link(link_) 
    { 
      parentId = -1; 
    }

    Link*             link;
    int               parentId;
    std::vector<int>  childId;
  };
     
  std::vector<_Node> _nodes;
  mrt::MultiBody*    _mb;
  bool               _isJoinedRoot;

  bool               _isInited;
};

} // end of namespace ab

#endif //_AB_ARTBODY_H_