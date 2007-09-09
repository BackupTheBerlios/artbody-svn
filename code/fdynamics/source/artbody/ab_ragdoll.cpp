//////////////////////////////////////////////////////////////////////////
// ab_f_imp.cpp
//////////////////////////////////////////////////////////////////////////

// standard
#include <math.h>
// this system
#include "ab_ragdoll.h"
#include "ab_joint.h"
#include "ab_link.h"

using namespace ab;

ab::_BodyIndices _fullModelIndicies[TOTAL_SEGMENTS_NUM] = {
    {LT, -1, 0, -1, ab::Vect4::UnitYNeg},    // 0 
    {MT, 0, 1, 0, ab::Vect4::UnitY},         // 1
    {UT, 1, 2, 0, ab::Vect4::UnitY},         // 2
    {HEAD, 2, 0, 0, ab::Vect4::UnitY},       // 3
    {LTHIGH, 0, 0, 2, ab::Vect4::UnitYNeg},  // 4
    {LSHANK, 4, 0, 1, ab::Vect4::UnitYNeg},  // 5
    {LFOOT, 5, 0, 1, ab::Vect4::UnitZ},      // 6
    {LUA, 2, 0, 3, ab::Vect4::UnitYNeg},     // 7
    {LFA, 7, 0, 1, ab::Vect4::UnitYNeg},     // 8
    {LHAND, 8, 0, 1, ab::Vect4::UnitYNeg},   // 9
    {RTHIGH, 0, 0, 1, ab::Vect4::UnitYNeg},  // 10
    {RSHANK, 10, 0, 1, ab::Vect4::UnitYNeg}, // 11
    {RFOOT, 11, 0, 1, ab::Vect4::UnitZ},     // 12
    {RUA, 2, 0, 1, ab::Vect4::UnitYNeg},     // 13
    {RFA, 13, 0, 1, ab::Vect4::UnitYNeg},    // 14
    {RHAND, 14, 0, 1, ab::Vect4::UnitYNeg}   // 15
};

ab::_BodyIndices _testModelIndicies[TOTAL_SEGMENTS_NUM] = {
    {LT, -1, 0, -1, ab::Vect4::UnitYNeg},    // 0 
    {MT, 0, 1, 0, ab::Vect4::UnitY},         // 1
    {UT, 1, 2, 0, ab::Vect4::UnitY},         // 2
    {HEAD, 2, 0, 0, ab::Vect4::UnitY},       // 3
    {LTHIGH, 0, 0, 2, ab::Vect4::UnitYNeg},  // 4
    {LSHANK, 4, 0, 1, ab::Vect4::UnitYNeg},  // 5
    {LFOOT, 5, 0, 1, ab::Vect4::UnitZ},      // 6
    {LUA, 2, 0, 3, ab::Vect4::UnitYNeg},     // 7
    {LFA, 7, 0, 1, ab::Vect4::UnitYNeg},     // 8
    {LHAND, 8, 0, 1, ab::Vect4::UnitYNeg},   // 9
    {RTHIGH, 0, 0, 1, ab::Vect4::UnitYNeg},  // 10
    {RSHANK, 10, 0, 1, ab::Vect4::UnitYNeg}, // 11
    {RFOOT, 11, 0, 1, ab::Vect4::UnitZ},     // 12
    {RUA, 2, 0, 1, ab::Vect4::UnitYNeg},     // 13
    {RFA, 13, 0, 1, ab::Vect4::UnitYNeg},    // 14
    {RHAND, 14, 0, 1, ab::Vect4::UnitYNeg}   // 15
};

RagDollModel::RagDollModel(int gender, int nation, Real weight, Real height, const Matr4& position, bool isTestBody)
{
    if (isTestBody) {
        memcpy(_partsIndices, _testModelIndicies, sizeof(_partsIndices));
        _artBody = _createTestBody(position);                
    } else {
        memcpy(_partsIndices, _fullModelIndicies, sizeof(_partsIndices));

        _artBody = new ArtBody();

        ANT_DESK desc = OpenInfo(gender, nation);

        ab::Matr4 transf;
        int nParts;
        _BodyPartParams part_params[TOTAL_SEGMENTS_NUM];

        ab::JointDOFParams dofsParams[3];    
        dofsParams[0].axis = ab::Vect4::UnitX;
        dofsParams[1].axis = ab::Vect4::UnitY;        
        dofsParams[2].axis = ab::Vect4::UnitZ;
        dofsParams[0].springK = 10;   // Z
        dofsParams[0].dumpK   = 1; // 
        dofsParams[1].springK = 10;   // Y
        dofsParams[1].dumpK   = 1; // 
        dofsParams[2].springK = 10;   // X
        dofsParams[2].dumpK   = 1; // 


        ab::Matr4 modelTrans = position;
        Link* pParent = NULL;
        ab::Real totalWeight = 0.0;
        for (int body_part_nmb = 0; body_part_nmb < TOTAL_SEGMENTS_NUM; body_part_nmb++) {
            part_params[body_part_nmb].desc = &desc;
            part_params[body_part_nmb].hum_height = height;
            part_params[body_part_nmb].hum_mass = weight;

            LinkParams lp;         
            part_params[body_part_nmb].bodyPart       = _partsIndices[body_part_nmb].partIdx;
            part_params[body_part_nmb].bodyPartParent = _partsIndices[body_part_nmb].parentIdx;
            part_params[body_part_nmb].parentJointIdx = _partsIndices[body_part_nmb].parentJointIdx;
            if (part_params[body_part_nmb].bodyPartParent != -1) {
                // calculate model matrix
                Link* parentLink = _partsArr[_partsIndices[body_part_nmb].parentIdx];
                _BodyPartParams& parentParams = part_params[_partsIndices[body_part_nmb].parentIdx];
                int              inParentJointIdx = _partsIndices[body_part_nmb].inParentJointIdx;
                ab::Vect4 trans = parentParams.joint_shitfs[inParentJointIdx];
                trans = Link::VectLocalFromMirtCoord(parentLink, trans);
                modelTrans = ab::Matr4::Identity;
                modelTrans.setTranslate(trans);
            }
            part_params[body_part_nmb].modelTrans = modelTrans;
            Link* pLink = _createBodyPart(part_params[body_part_nmb], _partsIndices[body_part_nmb].dir, part_params[body_part_nmb].parentJointIdx, lp, part_params[body_part_nmb].joint_shitfs, nParts);                
            _partsArr[body_part_nmb] = pLink;

            totalWeight += pLink->getParams().mass;

            if (body_part_nmb == 0) {
                _artBody->setRoot(pLink);
            } else {
                Joint* pJoint = _artBody->createJoint(3, dofsParams);
                pParent = _partsArr[_partsIndices[body_part_nmb].parentIdx];
                pJoint->connect(pParent, pLink);
            }
        }

    //    // TEST - setup initial pose
    //    ab::Vect4 cm;
    //    for (unsigned int i = 0; i < _artBody->getNLinks(); i++) {
    //        const LinkParams& p = _artBody->getLink(i)->getParams();
    //        cm = cm + (_artBody->getLink(i)->getLocalMatr() * p.centerMassPos) * (p.mass / totalWeight);
    //    }
    //
    //    
    //    const LinkParams& base_link_params = _artBody->getRoot()->getParams();
    //    ab::Vect4 ang_vel = ab::Vect4(1, 0, 0);
    //    ab::Vect4 lin_vel = ang_vel.cross(base_link_params.centerMassPos - cm);
    //    _artBody->getRoot()->setInitialState(0, 0, lin_vel, ang_vel);
    }

    _artBody->init();

//    _artBody->_mb->_l_l_1 = &_artBody->_mb->_links[1];
//    _artBody->_mb->_l_l_2 = &_artBody->_mb->_links[2];
//    _artBody->_mb->_l_l_3 = &_artBody->_mb->_links[3];
//    _artBody->_mb->_r_l_1 = &_artBody->_mb->_links[4];
//    _artBody->_mb->_r_l_2 = &_artBody->_mb->_links[5];
//    _artBody->_mb->_r_l_3 = &_artBody->_mb->_links[6];
}

void RagDollModel::getRenderLinks(RenderLinksVector &links) const
{
  unsigned numLinks = _artBody->getNLinks();
  links.resize(numLinks);

  for (unsigned i = 0; i < numLinks; ++i) 
  {
    Link* pObjLink = _artBody->getLink(i);
    LinkParams const &objLinkParams   = pObjLink->getParams();
    LinkRenderParams &curRenderParams = links[i];
    
    curRenderParams.matrGlobal = pObjLink->getLocalMatr();
    curRenderParams.matrLocal  = objLinkParams.modelMatr;
    curRenderParams.vert1      = objLinkParams.toRenderParams().vert1;
    curRenderParams.vert2      = objLinkParams.toRenderParams().vert2;
    curRenderParams.parentIdx  = _artBody->getParent(i);
  }

  return;
}

void RagDollModel::applyTorqueToRegion(int bodyPart, Vect4 torque)
{
    // find needed part    
    int internalLinkIdx = _findBodyPartLink(bodyPart);

    if (internalLinkIdx == -1) {
        // part not found
        return;
    }

    if (internalLinkIdx == 0) {
        // base link
        return;
    }

    _applyMomentToBodyPart(internalLinkIdx, torque);
}

void RagDollModel::applyForceToRegion(int bodyPart, Vect4 force)
{
    // find needed part    
    int internalLinkIdx = _findBodyPartLink(bodyPart);

    if (internalLinkIdx == -1) {
        // part not found
        return;
    }

    if (internalLinkIdx == 0) {
        // base link
        return;
    }

    Link* curLink = _partsArr[internalLinkIdx];
    Vect4 radVect = curLink->getLocalMatr().transformAsVector(curLink->getParams().centerMassPos);
    Vect4 moment  = force.cross(radVect);

    _applyMomentToBodyPart(internalLinkIdx, moment);
}

void RagDollModel::setPositionOfRegion(int antBodyPart, Vect4 positionVect)
{
    // find needed part    
    int internalLinkIdx = _findBodyPartLink(antBodyPart);

    if (internalLinkIdx == -1) {
        // part not found
        return;
    }

    if (internalLinkIdx == 0) {
        // base link
        return;
    }

    Link* curLink = _partsArr[internalLinkIdx];
    for (int dirNmb = 0; dirNmb < 3 && curLink; dirNmb++) {
        int posDirIdx = 0;
        if (dirNmb == 0) {
            posDirIdx = 2; // Z
        }
        if (dirNmb == 1) {
            posDirIdx = 0; // X
        }
        if (dirNmb == 2) {
            posDirIdx = 1; // Y
        }
        // set position for current DOF        
        _artBody->setDesiredPos(curLink, positionVect[posDirIdx]);
        // next link
        curLink = _artBody->getParent(curLink);
    }
}

void RagDollModel::setRegionSpringOffset(int antBodyPart, Vect4 springOffset)
{
    // find needed part    
    int internalLinkIdx = _findBodyPartLink(antBodyPart);

    if (internalLinkIdx == -1) {
        // part not found
        return;
    }

    if (internalLinkIdx == 0) {
        // base link
        return;
    }

    Link* curLink = _partsArr[internalLinkIdx];
    for (int dirNmb = 0; dirNmb < 3 && curLink; dirNmb++) {
        int posDirIdx = 0;
        if (dirNmb == 0) {
            posDirIdx = 2; // Z
        }
        if (dirNmb == 1) {
            posDirIdx = 0; // X
        }
        if (dirNmb == 2) {
            posDirIdx = 1; // Y
        }
        // set position for current DOF        
        _artBody->setSpringOffset(curLink, springOffset[posDirIdx]);
        // next link
        curLink = _artBody->getParent(curLink);
    }
}

//
// Internal methods
//

Link* RagDollModel::_createBodyPart(const _BodyPartParams& bodyParams,
                                             const Vect4& dir, 
                                             int toParentJointIdx, 
                                             LinkParams& lp,
                                             Vect4* joint_shitfs,                                      
                                             int& nParts)
{
    ANT_INFO inf;	
    float segment_bounds[15];

    FullSegmentInfo(*bodyParams.desc, bodyParams.bodyPart, bodyParams.hum_mass, bodyParams.hum_height, segment_bounds, &nParts, &inf);
    lp.lenght        = inf.length;
    lp.mass          = inf.mass;    
    lp.inertia       = Vect4( inf.iy, inf.iz, inf.ix );
    lp.dir           = dir;
    lp.modelMatr     = bodyParams.modelTrans;

    for (int i = 0; i < nParts; i++) {
        joint_shitfs[i].x = segment_bounds[i*3 + 1]; //y
        joint_shitfs[i].y = segment_bounds[i*3 + 2]; //z
        joint_shitfs[i].z = segment_bounds[i*3 + 0]; //x
    }

    lp.centerMassPos = joint_shitfs[toParentJointIdx];
    if (bodyParams.bodyPart != LFOOT && bodyParams.bodyPart != RFOOT) {
        lp.centerMassPos = lp.centerMassPos * -1;
    } 

    return new Link(lp);

}

int RagDollModel::_findBodyPartLink(int bodyPartIdx)
{
    for (int partNmb = 0; partNmb < TOTAL_SEGMENTS_NUM; partNmb++) {
        if (_partsIndices[partNmb].partIdx == bodyPartIdx) {
            return partNmb;
        }
    }
    return -1;
}

void RagDollModel::_applyMomentToBodyPart(int internalLinkIdx, Vect4 moment)
{
    Link* curLink = _partsArr[internalLinkIdx];
    for (int dirNmb = 0; dirNmb < 3 && curLink; dirNmb++) {
        // apply moment to current link
        const LinkParams& cur_link_params = curLink->getParams();
        Vect4 abs_joint_dir = curLink->getLocalMatr().transformAsVector(cur_link_params.jointAxis);
        Real  cur_joint_torque_value = abs_joint_dir.dot(moment);
        _artBody->applyMoment(curLink, cur_joint_torque_value);
        // next link
        curLink = _artBody->getParent(curLink);
    }
}

ArtBody* RagDollModel::_createTestBody(const Matr4& position)
{
    ab::ArtBody* mb = new ab::ArtBody();

    ab::Vect4 Body_Rt_Size(0, 0.518, -0.007);
    ab::Vect4 Body_01_Size(0, -0.284,  0);
    ab::Vect4 Body_01_IPos(0,  0,  0);
    ab::Vect4 Body_01_IVel(0,  0,  0);
    ab::Real  X_Ang_Correction = -0.014576679271515;

    // head
    ab::Real mh = 5.977001, ah = 0.145733, lh = -0.145733;
    ab::Real hIz = 0.038920, hIx = 0.041222, hIy = 0.023540;
    // ut
    ab::Real mut = 11.425000, aut = 0.166680, lut = -0.114400;
    ab::Real utIz = 0.129990, utIx = 0.082270, utIy = 0.104320;
    // mt
    ab::Real mmt = 7.645000, amt = 0.085999, lmt = -0.088441;
    ab::Real mtIz = 0.061490, mtIx = 0.039590, mtIy = 0.059500;
    // lt
    ab::Real mlt = 6.350001, alt = 0.128800, llt = -0.083703;
    ab::Real ltIz = 0.052000, ltIx = 0.039200, ltIy = 0.051200;

    ab::Real m_root = mh + mut + mmt + mlt;
    ab::Real d_u = aut; 
    ab::Real d_m = d_u -lut + amt;  
    ab::Real d_l = d_m -lmt + alt; 
    ab::Real a_root = (-lh*mh + d_u*mut + d_m*mmt + d_l*mlt)/m_root;
    ab::Real l_root = a_root - (d_l -llt);
    ab::Real r_Ix = (hIx + utIx + mtIx + ltIx);
    ab::Real r_Iz = (hIz + utIz + mtIz + ltIz);
    ab::Real r_Isqr = (mh*(a_root-lh)*(a_root-lh) + mut*(a_root-d_u)*(a_root-d_u) + mmt*(a_root-d_m)*(a_root-d_m) + mlt*(a_root-d_l)*(a_root-d_l));
    r_Ix = r_Ix + r_Isqr;
    ab::Real r_Iy = (hIy + utIy + mtIy + ltIy);
    r_Iz = r_Iz + r_Isqr;
    ab::Real Body_Rt_Mass = m_root;
    ab::Vect4 Body_Rt_Size_H(0, (a_root-lh+ah),  0);
    // UT
    ab::Vect4 Body_Rt_Size_ShR(-0.132358,  a_root,  0);
    ab::Vect4 Body_Rt_Size_ShL(0.132358,  a_root,  0);
    // LT
    ab::Vect4 Body_Rt_Size_HR(-0.083703,  l_root,  0);
    ab::Vect4 Body_Rt_Size_HL(0.083703,  l_root,  0);
    ab::Vect4 Body_Rt_Inertia(r_Ix, r_Iy, r_Iz);

    // RTHIGH
    ab::Real mTH = 7.240000, aTH = 0.144914, lTH = -0.271506;
    ab::Real THIz = 0.108330, THIx = 0.109780, THIy = 0.018430;
    // RSHANK
    ab::Real mSH = 3.310000, aSH = 0.180801, lSH = -0.220979;
    ab::Real SHIz = 0.038936, SHIx = 0.034070, SHIy = 0.008254;
    // RFOOT
    ab::Real mFt = 0.917000, aFt_y = 0.0, lFt_y = -0.0, aFt_z = -0.114202, lFt_z = -aFt_z;
    ab::Real FtIz = 0.000405, FtIx = 0.002230, FtIy = 0.002261;

    ab::Real m_rleg = mTH + mSH + mFt;
    ab::Real d_SH = aTH-lTH+aSH, d_Ft_y = d_SH -lSH + aFt_y, d_Ft_z = -aFt_z;
    ab::Real a_rleg_y = (aTH*mTH + d_SH*mSH + d_Ft_y*mFt)/m_rleg;
    ab::Real a_rleg_z = -d_Ft_z*mFt/m_rleg;
    ab::Real l_rleg_y = a_rleg_y - d_Ft_y;
    ab::Real l_rleg_z = lFt_z;
    r_Ix = (THIx + SHIx + FtIx);
    r_Iy = (THIy + SHIy + FtIy);
    r_Iz = (THIz + SHIz + FtIz);
    ab::Real sqr_Ix = mTH*(a_rleg_y-aTH)*(a_rleg_y-aTH) + mSH*(a_rleg_y-d_SH)*(a_rleg_y-d_SH) + mFt*(d_Ft_z*d_Ft_z+l_rleg_y*l_rleg_y);
    ab::Real sqr_Iy = mFt*d_Ft_z*d_Ft_z;
    ab::Real sqr_Iz = mTH*(a_rleg_y-aTH)*(a_rleg_y-aTH) + mSH*(a_rleg_y-d_SH)*(a_rleg_y-d_SH) + mFt*l_rleg_y*l_rleg_y;
    r_Ix = r_Ix + sqr_Ix;
    r_Iy = r_Iy + sqr_Iy;
    r_Iz = r_Iz + sqr_Iz;

    ab::Real Leg_R_Mass = m_rleg;
    ab::Vect4 Leg_R_Size_a(0, a_rleg_y, a_rleg_z);
    ab::Vect4 Leg_R_Size_l(0, l_rleg_y, -a_rleg_z);
    ab::Vect4 Leg_R_Inertia(r_Ix, r_Iy, r_Iz);

    // LTHIGH
    // LSHANK
    // LFOOT
    ab::Real  Leg_L_Mass = Leg_R_Mass;
    ab::Vect4 Leg_L_Size_a = Leg_R_Size_a;
    ab::Vect4 Leg_L_Size_l = Leg_R_Size_l;
    ab::Vect4 Leg_L_Inertia = Leg_R_Inertia;

    // RUA
    mTH = 1.443000; aTH = 0.112950; lTH = -0.138050;
    THIz = 0.008078; THIx = 0.008360; THIy = 0.001472;
    // RFA
    mSH = 0.950000; aSH = 0.108631; lSH = -0.142829;
    SHIz = 0.004635; SHIx = 0.004516; SHIy = 0.000583;
    // RHAND
    mFt = 0.349000; 
    ab::Real aFt = 0.065858; 
    ab::Real lFt = -aFt;
    FtIz = 0.000389; FtIx = 0.000482; FtIy = 0.000176;

    m_rleg = mTH + mSH + mFt;
    d_SH = aTH-lTH+aSH;
    ab::Real d_Ft = d_SH -lSH + aFt;
    ab::Real a_rleg = (aTH*mTH + d_SH*mSH + d_Ft*mFt)/m_rleg;
    ab::Real l_rleg = a_rleg - d_Ft;
    r_Ix = (THIx + SHIx + FtIx);
    r_Iy = (THIy + SHIy + FtIy);
    r_Iz = (THIz + SHIz + FtIz);
    sqr_Ix = mTH*(a_rleg-aTH)*(a_rleg-aTH) + mSH*(a_rleg-d_SH)*(a_rleg-d_SH) + mFt*l_rleg*l_rleg;
    sqr_Iy = 0;
    sqr_Iz = sqr_Ix;
    r_Ix = r_Ix + sqr_Ix;
    r_Iy = r_Iy + sqr_Iy;
    r_Iz = r_Iz + sqr_Iz;

    ab::Real  Hnd_R_Mass = m_rleg;
    ab::Vect4 Hnd_R_Size_a(0, a_rleg,  0);
    ab::Vect4 Hnd_R_Size_l(0,  l_rleg,  0);
    ab::Vect4 Hnd_R_Inertia(r_Ix, r_Iy, r_Iz);

    // LUA
    // LFA
    // LHAND
    ab::Real  Hnd_L_Mass = Hnd_R_Mass;
    ab::Vect4 Hnd_L_Size_a = Hnd_R_Size_a;
    ab::Vect4 Hnd_L_Size_l = Hnd_R_Size_l;
    ab::Vect4 Hnd_L_Inertia = Hnd_R_Inertia;

    ab::Vect4 Body_CM_pos = ((Body_Rt_Size_ShR-Hnd_R_Size_a) * Hnd_R_Mass + 
        (Body_Rt_Size_ShL-Hnd_L_Size_a) * Hnd_L_Mass + 
        (Body_Rt_Size_HR -Leg_R_Size_a) * Leg_R_Mass + 
        (Body_Rt_Size_HL -Leg_L_Size_a) * Leg_L_Mass) * 
        (1 / (Body_Rt_Mass+Hnd_R_Mass+Hnd_L_Mass+Leg_R_Mass+Leg_L_Mass));    
    ab::Vect4 Body_Rt_IPos =  Body_CM_pos * -1;
    ab::Vect4 Body_Rt_IVel_Rot(0,  0,  0);    
    ab::Vect4 Body_Rt_IVel_Transl = Body_Rt_IVel_Rot.cross(Body_CM_pos) * -1;

    ab::Link* body;
    ab::Link* left_leg;
    ab::Link* right_leg;
    ab::Link* left_arm;
    ab::Link* right_arm;

    // body
    ab::LinkParams body_lp;
    body_lp.mass = Body_Rt_Mass;
    body_lp.inertia = Body_Rt_Inertia;
    body_lp.lenght        =  2 * Body_Rt_Size.norm();
    body_lp.centerMassPos =  Body_Rt_Size * -1;
    body_lp.modelMatr     =  position;
    body_lp.dir           =  ab::Vect4::UnitYNeg;
    body_lp.base_ang_vel  =  Body_Rt_IVel_Rot;
    body_lp.base_lin_vel  =  Body_Rt_IVel_Transl;

    body = new ab::Link(body_lp);

    // left leg
    ab::LinkParams left_leg_lp;
    left_leg_lp.mass = Leg_L_Mass;
    left_leg_lp.inertia = Leg_L_Inertia;
    left_leg_lp.lenght        =  2 * Leg_L_Size_a.norm();
    left_leg_lp.centerMassPos =  Leg_L_Size_a * -1;
    left_leg_lp.modelMatr.translate(ab::Link::VectLocalFromMirtCoord(body, Body_Rt_Size_HL));
    left_leg_lp.modelMatr.rotateX(ab::DegToRad(-0.014576679271515));
    left_leg_lp.dir           =  ab::Vect4::UnitYNeg;

    left_leg = new ab::Link(left_leg_lp);

    // right leg
    ab::LinkParams right_leg_lp;
    right_leg_lp.mass = Leg_R_Mass;
    right_leg_lp.inertia = Leg_R_Inertia;
    right_leg_lp.lenght        =  2 * Leg_R_Size_a.norm();
    right_leg_lp.centerMassPos =  Leg_R_Size_a * -1;
    right_leg_lp.modelMatr.translate(ab::Link::VectLocalFromMirtCoord(body, Body_Rt_Size_HR));
    right_leg_lp.modelMatr.rotateX(ab::DegToRad(-0.014576679271515));
    right_leg_lp.dir           =  ab::Vect4::UnitYNeg;

    right_leg = new ab::Link(right_leg_lp);

    // left arm
    ab::LinkParams left_arm_lp;
    left_arm_lp.mass = Hnd_L_Mass;
    left_arm_lp.inertia = Hnd_L_Inertia;
    left_arm_lp.lenght        =  2 * Hnd_L_Size_a.norm();
    left_arm_lp.centerMassPos =  Hnd_L_Size_a * -1;
    left_arm_lp.modelMatr.translate(ab::Link::VectLocalFromMirtCoord(body, Body_Rt_Size_ShL));
    left_arm_lp.modelMatr.rotateX(ab::DegToRad(-0.014576679271515));
    left_arm_lp.dir           =  ab::Vect4::UnitYNeg;

    left_arm = new ab::Link(left_arm_lp);

    // right arm
    ab::LinkParams right_arm_lp;
    right_arm_lp.mass = Hnd_R_Mass;
    right_arm_lp.inertia = Hnd_R_Inertia;
    right_arm_lp.lenght        =  2 * Hnd_R_Size_a.norm();
    right_arm_lp.centerMassPos =  Hnd_R_Size_a * -1;
    right_arm_lp.modelMatr.translate(ab::Link::VectLocalFromMirtCoord(body, Body_Rt_Size_ShR));
    right_arm_lp.modelMatr.rotateX(ab::DegToRad(-0.014576679271515));
    right_arm_lp.dir           =  ab::Vect4::UnitYNeg;

    right_arm = new ab::Link(right_arm_lp);

    // joint
    ab::JointDOFParams dofsParams[3];    
    dofsParams[0].axis = ab::Vect4::UnitZ;
    dofsParams[1].axis = ab::Vect4::UnitY;
    dofsParams[2].axis = ab::Vect4::UnitX;

    dofsParams[0].springK = 6;   // Z
    dofsParams[0].dumpK   = /*0.3*/1; // 
    dofsParams[1].springK = 15;   // Y
    dofsParams[1].dumpK   = /*0.3*/1; // 
    dofsParams[2].springK = 15;   // X
    dofsParams[2].dumpK   = /*0.5*/1.5; // 
    ab::Joint* body_left_leg = mb->createJoint(3, dofsParams);
    dofsParams[0].springK = 6;   // Z
    dofsParams[0].dumpK   = /*0.3*/1; // 
    dofsParams[1].springK = 15;   // Y
    dofsParams[1].dumpK   = /*0.3*/1; // 
    dofsParams[2].springK = 15;   // X
    dofsParams[2].dumpK   = /*0.5*/1.5; // 
    ab::Joint* body_right_leg = mb->createJoint(3, dofsParams);
    dofsParams[0].springK = 6;   // Z
    dofsParams[0].dumpK   = /*0.3*/1; // 
    dofsParams[1].springK = 15;   // Y
    dofsParams[1].dumpK   = /*0.3*/1; // 
    dofsParams[2].springK = 15;   // X
    dofsParams[2].dumpK   = /*0.5*/1.5; // 
    ab::Joint* body_left_arm = mb->createJoint(3, dofsParams);
    dofsParams[0].springK = 6;   // Z
    dofsParams[0].dumpK   = /*0.3*/1; // 
    dofsParams[1].springK = 15;   // Y
    dofsParams[1].dumpK   = /*0.3*/1; // 
    dofsParams[2].springK = 15;   // X
    dofsParams[2].dumpK   = /*0.5*/1.5; // 
    ab::Joint* body_right_arm = mb->createJoint(3, dofsParams);
    mb->setRoot(body);
    body_left_leg->connect(body, left_leg);
    body_right_leg->connect(body, right_leg);
    body_left_arm->connect(body, left_arm);
    body_right_arm->connect(body, right_arm);

    // body
    _partsArr[0] = _partsArr[1] = _partsArr[2] = _partsArr[3] = body;
    _partsArr[4] = _partsArr[5] = _partsArr[6] = left_leg;
    _partsArr[7] = _partsArr[8] = _partsArr[9] = left_arm;
    _partsArr[10] = _partsArr[11] = _partsArr[12] = right_leg;
    _partsArr[13] = _partsArr[14] = _partsArr[15] = right_arm;

    return mb;
}