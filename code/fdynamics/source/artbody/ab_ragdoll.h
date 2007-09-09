#ifndef _AB_RAGDOLL_H_
#define _AB_RAGDOLL_H_

// standard
#include <assert.h>
// STL
#include <vector>
// This system
#include "ab_types.h"
#include "ab_link.h"
#include "ab_artbody.h"
#include "ab_anthropometric.h"

namespace ab
{

// internal types
struct _BodyIndices {
    int         partIdx;
    int         parentIdx;
    int         parentJointIdx;
    int         inParentJointIdx;
    ab::Vect4   dir;
};

class RagDollModel
{
public:
    RagDollModel(int gender, int nation, Real weight, Real height, const Matr4& position, bool isTestBody = false);
    ~RagDollModel()
    {
        assert(_artBody);
        delete _artBody;
        _artBody = NULL;
    }

    // process
    void update(Real elapsed) { _artBody->processTimer(elapsed); }

    typedef std::vector<LinkRenderParams> RenderLinksVector;
    // get
    void getRenderLinks(RenderLinksVector &links) const;
    mrt::MultiBody* getMultiBody() const { return _artBody->getMultiBody(); }
    ab::ArtBody*    getArtBody  ()       { return _artBody; }

    // management    
    void applyTorqueToRegion  (int antBodyPart, Vect4 torque);
    void applyForceToRegion   (int antBodyPart, Vect4 force);
    void setPositionOfRegion  (int antBodyPart, Vect4 positionVect);
    void setRegionSpringOffset(int antBodyPart, Vect4 springOffset);
private:
    struct _BodyPartParams {
        explicit _BodyPartParams() {;}
        explicit _BodyPartParams(const ab::ANT_DESK& desc_, ab::Real height, ab::Real mass);

        const ab::ANT_DESK* desc;
        ab::Real hum_height;
        ab::Real hum_mass;
        int bodyPart;        
        ab::Matr4 modelTrans;                        
        int bodyPartParent;        
        int parentJointIdx;

        ab::Vect4 joint_shitfs[4];
    };

    _BodyIndices _partsIndices[TOTAL_SEGMENTS_NUM];

    static Link*  _createBodyPart  (const _BodyPartParams& bodyParams,
                                        const Vect4& dir, 
                                        int toParentJointIdx, 
                                        LinkParams& lp,
                                        Vect4* joint_shitfs,                                
                                        int& nParts);
    int          _findBodyPartLink     (int bodyPartIdx);
    void         _applyMomentToBodyPart(int internalLinkIdx, Vect4 moment);
    ArtBody*     _createTestBody       (const Matr4& position);

    // Articulated structure
    ArtBody* _artBody;
    Link*    _partsArr[TOTAL_SEGMENTS_NUM];
};

} // end of namespace ab

#endif //_AM_F_IMP_H_