#ifndef _AB_JOINT_H_
#define _AB_JOINT_H_

#include "ab_types.h"


namespace ab {

class Link;
class ArtBody;
class Vect4;

enum {
    JointAxisX = 0x0001,
    JointAxisY = 0x0002,
    JointAxisZ = 0x0004
};

struct JointDOFParams {
    JointDOFParams():axis(Vect4::UnitZ), springK(0), dumpK(0) {}
    Vect4  axis;
    Real   springK;
    Real   dumpK;
};

class Joint {
friend class ArtBody;
public:
    virtual ~Joint();

    bool connect(Link* inboard, Link* outboard);
private:
    explicit Joint(int nDOF, JointDOFParams* dofsParams, ArtBody* pOwner); //JointAxisX, JointAxisY, JointAxisZ

    Link*           _links[3];
    int             _nInternalLinks;
    int             _nDOF;

    JointDOFParams  _explicitJoint;

    ArtBody*        _pOwner;
    int             _inLinkIdx;
    int             _outLinkIdx;
};


} // namespace ab

#endif //_AB_JOINT_H_
