//////////////////////////////////////////////////////////////////////////
// Processor.cpp
//////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <gl/glut.h>
#include <math.h>

#include "ChainApp.h"
//#include "la.h"
#include "render.h"
#include "timer.h"
#include "camera.h"
#include "processor.h"
//#include "adapter.h"
//#include "obj.h"
//#include "link_obj.h"
#include "result_grabber.h"

#include "ab_anthropometric.h"
#include "ab_ragdoll.h"
#include "ab_artbody.h"
#include "ab_joint.h"
#include "ab_vect4.h"
#include "ab_matr4.h"

//
// Local classes
// 
namespace app {

struct PendulumSimulationResult// : public res::ReslultEntry 
{
    float time;
    float angle;
    float velocity;
    float accel;
    int serialize(char ** buff) {
        char tmpBuff[512];        
        sprintf(tmpBuff, "%f %f %f %f\n", time, angle, velocity, accel);
        int size = strlen(tmpBuff) + 1;
        *buff = (char*)calloc(size, 1);
        if (*buff == NULL) {
            return 0;
        }
        strcpy(*buff, tmpBuff);
        return size;
    }
};

struct FPSSimulationResult
{
    float time;
    int serialize(char ** buff) {
        char tmpBuff[512];        
        sprintf(tmpBuff, "%e\n", time);
        int size = strlen(tmpBuff) + 1;
        *buff = (char*)calloc(size, 1);
        if (*buff == NULL) {
            return 0;
        }
        strcpy(*buff, tmpBuff);
        return size;
    }
};

struct CMSimultinResult {
    int         frameNmb;
    int         nLinks;
    ab::Vect4   linksCOM[100];
    int serialize(char ** buff) {
        char tmpBuff[2048];        
        tmpBuff[0] = 0;
        sprintf(tmpBuff, "%i\t", frameNmb);
        for (int linkIdx = 0; linkIdx < nLinks; linkIdx++) {
            char linkTmpBuff[128];
            sprintf(linkTmpBuff, "%f %f %f\t", linksCOM[linkIdx].x, linksCOM[linkIdx].y, linksCOM[linkIdx].z);
            strcat(tmpBuff, linkTmpBuff);
        }
        strcat(tmpBuff, "\n");
        int size = strlen(tmpBuff) + 1;
        *buff = (char*)calloc(size, 1);
        if (*buff == NULL) {
            return 0;
        }
        strcpy(*buff, tmpBuff);
        return size;
    }
};

struct BodyAccSimulationResult 
{
    int frameNmb;
    int nLinks;
    ab::Real linkAcc[100];

    int serialize(char ** buff) {
        char tmpBuff[2048];        
        tmpBuff[0] = 0;
        sprintf(tmpBuff, "%i\t", frameNmb);
        for (int linkIdx = 0; linkIdx < nLinks; linkIdx++) {
            char linkTmpBuff[128];
            sprintf(linkTmpBuff, "%10.4e\t", linkAcc[linkIdx]);
            strcat(tmpBuff, linkTmpBuff);
        }
        strcat(tmpBuff, "\n");
        int size = strlen(tmpBuff) + 1;
        *buff = (char*)calloc(size, 1);
        if (*buff == NULL) {
            return 0;
        }
        strcpy(*buff, tmpBuff);
        return size;
    }
};


res::ResultsGrabber<PendulumSimulationResult> _resultsGrabber;
res::ResultsGrabber<FPSSimulationResult> _fpsResultsGrabber;
res::ResultsGrabber<CMSimultinResult> _comResultsGrabber;
res::ResultsGrabber<BodyAccSimulationResult> _accResultsGrabber;

// controller
class Controller {
public:
    Controller(ab::Real intergationStep, ab::Real acc, ab::Real velBound);

    void       setDesiredValue(ab::Real value);

    ab::Real   getCurrentPosition(void) { return _curPos; }
    void       processStep();

private:
    ab::Real   _integrate(ab::Real curValue, ab::Real deltaValue, ab::Real deltaT, ab::Real sign, ab::Real bound);

    ab::Real   _integrationStep;
    
    ab::Real   _velBound;
    ab::Real   _desiredPos;

    ab::Real   _curAcc;
    ab::Real   _curVel;
    ab::Real   _curPos;        
};

Controller zArmAngController(0.01, 30, 100);

}


using namespace app;

Processor::Processor() :
_inputStage(_INPUT_NOT_PROCESS),
_isIdle(false),
_testMb(NULL),
_pRagDoll(NULL),
_testID(_TestTypeNone)
{
    //_createChains();    
    //_createTestBodies();
    //_createTestJointBodies();

    _createTestJointBodies_1();
    //_createTestJointBodies_2();
    //_createTestJointBodies_3();
    
    {
        // ab::Matr4 position;
        // position.translate(ab::Vect4(0, 0, 0));
        // _pRagDoll = new ab::RagDollModel(ab::FEMALE, ab::GER, 57.0, 1.779, position, true);
        // _testID = _TestTypeRagDoll;
    }
}

Processor::~Processor()
{
    ;
}

void Processor::onTimer(ab::Real elapsed)
{
    if (_isIdle) {
        return;
    }

    {
        ab::Real frame_time = _timer->getCurTime();
        //_mbManager.processTimer(elapsed);
        frame_time = _timer->getCurTime() - frame_time;

        

        if (_pRagDoll) {
            for (int i = 0; i < 5; i++) {
                zArmAngController.processStep();
                ab::Real pos = zArmAngController.getCurrentPosition();
                _pRagDoll->setRegionSpringOffset(ab::LTHIGH, ab::Vect4(ab::DegToRad(pos), 0, 0));
                _pRagDoll->setRegionSpringOffset(ab::RUA, ab::Vect4(0, 0, -1 * ab::DegToRad(pos)));            
                _pRagDoll->update(0.002);
            }
        }

        if (_testMb) {
            for (int i = 0; i < 1; i++) {
                zArmAngController.processStep();
            }            
            ab::Real pos = zArmAngController.getCurrentPosition();
//            if (!ab::isZero(pos)) {
//                __asm int 3;
//            }
            _testMb->setSpringOffset(_testMb->getLink(3), ab::DegToRad(pos));
//            _testMb->setSpringOffset(_testMb->getLink(5), ab::DegToRad(pos));
            for (int i = 0; i < 1; i++) {
                _testMb->processTimer(0.01);
            }            
        }
        
        // save results
        // FPS
        static unsigned long frameNmb = 0;
        static ab::Real allTime = 0.0;
        allTime += frame_time;
        if ((frameNmb % 100) == 0) {
            FPSSimulationResult res;
            res.time = allTime / 100.0 ;
            _fpsResultsGrabber.addResult(res);
            allTime = 0;
        }
        // CM pos
        {
//            if (_testMb) {
//                obj::MultiBody* mb = _testMb;
//                CMSimultinResult com_res;
//                com_res.frameNmb = frameNmb;
//                com_res.nLinks = mb->getNLinks();
//                for (unsigned int linkIdx = 0; linkIdx < mb->getNLinks(); linkIdx++) {
//                    obj::Link* pCurLink = mb->getLink(linkIdx);
//                    const obj::LinkParams& link_params = pCurLink->getParams();
//                    ab::Vect4 abs_com_pos = pCurLink->getLocalMatr() * link_params.centerMassPos;
//                    com_res.linksCOM[linkIdx] = abs_com_pos;
//                }
//                _comResultsGrabber.addResult(com_res);
//            }
            
        }

        // links ACC
        {
            if (_testMb) {
                BodyAccSimulationResult accRes;
                accRes.frameNmb = frameNmb;
                accRes.nLinks = _testMb->getNLinks() - 1;
                for (int linkIdx = 1; linkIdx < _testMb->getNLinks(); linkIdx++) {
                    ab::Link* curLink = _testMb->getLink(linkIdx);
                    const ab::LinkParams& params = curLink->getParams();
                    accRes.linkAcc[linkIdx - 1] = params.generalized_coord_p_p;
                }
                _accResultsGrabber.addResult(accRes);
            }
        }

        frameNmb++;
        if (frameNmb == 60) {
//            __asm int 3;
        }
        
    }
    return;                                
}

void Processor::onRender()
{      
       //glColor4ub(0xFF, 0xFF, 0xFF, 0xFF);
       //glLoadIdentity();
       //glTranslatef(0.f, 0.5f, 1.1f);
       //glutWireCube (0.5);

      //_renderer->renderSphereWire(ab::Vect4::Zero, 0.5f);
    
//    {
//        //_renderer->renderText(ab::Vect4(20.0, 20.0, -1.0), "AAAAA");
//        glLoadIdentity();
//        float pos[4];
//        glGetFloatv(GL_CURRENT_RASTER_POSITION, pos);
//        glDisable(GL_LIGHTING);        
//        glRasterPos2d(0, 0);
//        glGetFloatv(GL_CURRENT_RASTER_POSITION, pos);
//        for (int i = 0; i < 10; i++) {
//            glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 'A');
//            glGetFloatv(GL_CURRENT_RASTER_POSITION, pos);
//        }
//        glEnable(GL_LIGHTING);
//
//        
//        glGetFloatv(GL_CURRENT_RASTER_POSITION, pos);
//        GLboolean fValidPos;
//        glGetBooleanv(GL_CURRENT_RASTER_POSITION_VALID, &fValidPos);
//        fValidPos = true;
//    }

    _renderer->renderAxis(ab::Matr4::Identity, 1.5);

    {
        //_mbManager.processRender(*_renderer);
    }

    if (_testMb) {        
        ab::Real cm_mass = 0;
        for (int link_nmb = 0; link_nmb < _testMb->getNLinks(); link_nmb++) {
            ab::Link* curLink = _testMb->getLink(link_nmb);
            ab::LinkParams lp = curLink->getParams();
            ab::LinkRenderParams render_lp = lp.toRenderParams();
            const ab::Matr4& globalMatr = curLink->getLocalMatr();
            ab::Vect4 v1 = globalMatr * render_lp.vert1;
            ab::Vect4 v2 = globalMatr * render_lp.vert2;
            ab::Vect4 cm = globalMatr * lp.centerMassPos;
            _renderer->renderLine(v1, v2);
            _renderer->renderSphereWire(v1, 0.05, 0xFFFF0000);
            _renderer->renderSphereWire(v2, 0.05, 0xFFFF0000);
            _renderer->renderSphereWire(cm, 0.05, 0xFF0000FF);
            cm_mass += lp.mass;
        }

        // render CM
        ab::Vect4 cm_pos;
        for (int link_nmb = 0; link_nmb < _testMb->getNLinks(); link_nmb++) {
            ab::Link* curLink = _testMb->getLink(link_nmb);
            ab::LinkParams lp = curLink->getParams();
            const ab::Matr4& globalMatr = curLink->getLocalMatr();
            ab::Vect4 cm_rel = globalMatr * lp.centerMassPos * (lp.mass / cm_mass);
            cm_pos = cm_pos + cm_rel;
        }
        _renderer->renderSphereWire(cm_pos, 0.05, 0xFF00FF00);
    }
    

    {
        if (_pRagDoll) {
            ab::RagDollModel::RenderLinksVector links_vector;
            _pRagDoll->getRenderLinks(links_vector);
            for (int i = 0; i < links_vector.size(); i++) {
                ab::LinkRenderParams& curLink = links_vector[i];
                ab::Vect4 abv1 = curLink.matrGlobal * curLink.vert1;
                ab::Vect4 abv2 = curLink.matrGlobal * curLink.vert2;
                ab::Vect4 v1(abv1.x, abv1.y, abv1.z);
                ab::Vect4 v2(abv2.x, abv2.y, abv2.z);
                _renderer->renderLine(v1, v2);
                _renderer->renderSphereWire(v1, 0.05, 0xFFFF0000);
                _renderer->renderSphereWire(v2, 0.05, 0xFFFF0000);
            }
        }        
    }

    return;
}

void Processor::onInput(const inp::InputParams& params)
{
    if (_handleSwitchInputStage(params)) {
        return;
    }

    switch (_inputStage) {
      case _INPUT_NOT_PROCESS:
          break;
      case _INPUT_CAMERA:
          _cameraMng.handleCameraInput(params);
          break;
      case _INPUT_ACT:
          //_mbManager.handleInput(params);
          break;
    }
    
    // TEST
    switch (_testID) {
        case _TestTypeRagDoll:
            if (params.sym == 32 ) { // ' '
                if (_pRagDoll) {            
                    _pRagDoll->applyTorqueToRegion(ab::LUA, ab::Vect4(0, 0, 20));
                }                
            }
            break;
        case _TestTypeTest1:
            if (params.sym == 32 ) { // ' '
                static int coeff = 1;
                zArmAngController.setDesiredValue(150 * coeff);
                coeff = coeff ? 0 : 1;
            }
            break;
        case _TestTypeTest2:
            if (params.sym == 32 ) { // ' '
                if (_testMb) {            
                    _testMb->applyMoment(1, 5);
                }                            
            }
            break;            
    } //switch

    return;
}
// 
// Private functions
//
bool Processor::_handleSwitchInputStage(const inp::InputParams& params)
{
    switch (params.sym) {
      case 'N':
      case 'n':
          _inputStage = _INPUT_NOT_PROCESS;
          break;
      case 'C':
      case 'c':
          _inputStage = _INPUT_CAMERA;
          break;
      case 'a':
      case 'A':
          _inputStage = _INPUT_ACT;
          break;
      case 'i':
      case 'I':
          _isIdle = (_isIdle ? false : true);
          break;
      case 'q':
      case 'Q':
          App.Term();
          break;
      case 's':
      case 'S':
          // _resultsGrabber->flushResultsToFile("D:\\private\\phd\\samples\\pendulum\\Matlab7\\double_pendulum_4.txt");
          // _fpsResultsGrabber->flushResultsToFile("Time.txt");
          //_comResultsGrabber.flushResultsToFile("D:\\Temp\\COM_positions.txt");
          _accResultsGrabber.flushResultsToFile("D:\\Temp\\PHD\\Results\\acc_body_good_3.txt");
          break;
    }

    return false;
}

void Processor::_createTestBodies(void)
{
//    obj::MultiBody * mb;// = _mbManager.createMultiBody();
//
//    ab::Matr4 modelMatr;
//
//    
//    obj::LinkParams lp;
//    lp.lenght        = 1.0;                                      
//    lp.mass          = 1.0;
//    lp.inertia       = ab::Vect4(lp.lenght * lp.lenght, 0, lp.lenght * lp.lenght) * (lp.mass / 12.0);
//    lp.centerMassPos = ab::Vect4(0.0, -lp.lenght / 2.0, 0.0);
//    lp.modelMatr     = modelMatr;
//    lp.dir           = ab::Vect4::UnitYNeg;
//    lp.jointAxis     = ab::Vect4::UnitZ;
//    obj::Link* link1 = new obj::Link(lp);
//    mb->addChild(mb->getRoot(), link1);
//
////    modelMatr.translate(ab::Vect4(0.0, -lp.lenght, 0.0));
////    modelMatr.rotateY(ab::PI / 2.0, false);
////    obj::Link* link1_null1 = MultiBodyManager::createNullJoint(modelMatr);
////    mb->addChild(link1, link1_null1);
//
////    obj::Link* link_old = link1_null1;    
//      obj::Link* link_old = link1;    
//    modelMatr = ab::Matr4::Identity;
//    modelMatr.translate(ab::Vect4(0.0, -lp.lenght, 0.0));
//    lp.modelMatr = modelMatr;
//    for (int i = 0; i < 2; i++) {
//        obj::Link* link_new;
////        link_new = createNullJoint(modelMatr);
////        mb->addChild(link_old, link_new);
////        link_old = link_new;
//        if (i == 2) {
//            lp.mass     = 100.0;
//            lp.inertia  = ab::Vect4(lp.lenght * lp.lenght, 0, lp.lenght * lp.lenght) * (lp.mass / 12.0);
//        }
//        link_new = new obj::Link(lp);
//        mb->addChild(link_old, link_new);
//        link_old = link_new;  
//    }
//
//    mb->init();

    return;
}

void Processor::_createTestJointBodies(void)
{    
    ab::ArtBody* mb = new ab::ArtBody();

    ab::Link* l1;
    ab::Link* l2;
//    ab::Link* l3;
//    ab::Link* l4;
//    ab::Link* l5;
    ab::LinkParams lp1, lp2, lp3, lp4, lp5;
    
    lp1.mass = 10;
    lp1.inertia = ab::Vect4(lp1.mass / 12., 0.05, lp1.mass / 12.);
    lp1.lenght = 1;
    lp1.centerMassPos = ab::Vect4(0, -0.5, 0);
    lp1.dir = ab::Vect4::UnitYNeg;
    l1 = new ab::Link(lp1);

    lp2.mass = 10;
    lp2.inertia = ab::Vect4(lp2.mass /12., 0.05,  lp2.mass / 12.);
    lp2.lenght = 1;
    lp2.centerMassPos = ab::Vect4(0, -0.5, 0);
    lp2.dir = ab::Vect4::UnitYNeg;
    lp2.modelMatr.translate(ab::Link::VectLocalFromMirtCoord(l1, ab::Vect4(0.2, -0.5, 0)));
    l2 = new ab::Link(lp2);
//
//    lp3.mass = 10;
//    lp3.inertia = ab::Vect4(lp1.mass / 12., 0.00001, lp1.mass / 12.);
//    lp3.lenght = 1;
//    lp3.centerMassPos = ab::Vect4(0, -0.5, 0);
//    lp3.dir = ab::Vect4::UnitYNeg;
//    lp3.modelMatr.translate(ab::Link::VectLocalFromMirtCoord(l1, ab::Vect4(-0.2, -0.5, 0)));
//    l3 = new ab::Link(lp3);
//
//    lp4.mass = 10;
//    lp4.inertia = ab::Vect4(lp1.mass /12., 0.00001,  lp1.mass / 12.);
//    lp4.lenght = 1;
//    lp4.centerMassPos = ab::Vect4(0, -0.5, 0);
//    lp4.dir = ab::Vect4::UnitYNeg;
//    lp4.modelMatr.translate(ab::Link::VectLocalFromMirtCoord(l1, ab::Vect4(0.2, 0.5, 0)));
//    l4 = new ab::Link(lp4);
//
//    lp5.mass = 10;
//    lp5.inertia = ab::Vect4(lp1.mass / 12., 0.00001, lp1.mass / 12.);
//    lp5.lenght = 1;
//    lp5.centerMassPos = ab::Vect4(0, -0.5, 0);
//    lp5.dir = ab::Vect4::UnitYNeg;
//    lp5.modelMatr.translate(ab::Link::VectLocalFromMirtCoord(l1, ab::Vect4(-0.2, 0.5, 0)));
//    l5 = new ab::Link(lp5);
//
    ab::JointDOFParams dofsParams[3];    
    dofsParams[0].axis = ab::Vect4::UnitY;
    dofsParams[1].axis = ab::Vect4::UnitZ; //ab::Vect4::UnitY
    dofsParams[2].axis = ab::Vect4::UnitX;
//    dofsParams[0].springK = 10;   // Z
//    dofsParams[0].dumpK   = 1; // 
//    dofsParams[1].springK = 10;   // Y
//    dofsParams[1].dumpK   = 1; // 
//    dofsParams[2].springK = 10;   // X
//    dofsParams[2].dumpK   = 1; // 

    ab::Joint* j_1_2 = mb->createJoint(3, dofsParams);
//    ab::Joint* j_1_3 = mb->createJoint(3, dofsParams);
//    ab::Joint* j_1_4 = mb->createJoint(3, dofsParams);
//    ab::Joint* j_1_5 = mb->createJoint(3, dofsParams);
    mb->setRoot(l1);
    j_1_2->connect(l1, l2);
//    j_1_3->connect(l1, l3);
//    j_1_4->connect(l1, l4);
//    j_1_5->connect(l1, l5);
//
//    // setup initial velocity
//    ab::Real totalMass_inv =  1.0 / (lp1.mass + lp2.mass + lp3.mass + lp4.mass + lp5.mass);
//    ab::Real rel_L1_mass = lp1.mass * totalMass_inv;
//    ab::Real rel_L2_mass = lp2.mass * totalMass_inv;
//    ab::Real rel_L3_mass = lp3.mass * totalMass_inv;
//    ab::Real rel_L4_mass = lp4.mass * totalMass_inv;
//    ab::Real rel_L5_mass = lp5.mass * totalMass_inv;
//
//    ab::Vect4 cm_pos = (l1->getLocalMatr() * lp1.centerMassPos * lp1.mass + 
//                       l2->getLocalMatr() * lp2.centerMassPos * lp2.mass + 
//                       l3->getLocalMatr() * lp3.centerMassPos * lp3.mass +
//                       l4->getLocalMatr() * lp4.centerMassPos * lp4.mass + 
//                       l5->getLocalMatr() * lp5.centerMassPos * lp5.mass ) * totalMass_inv;
//    ab::Vect4 global_L1_CM_pos = (l1->getLocalMatr() * lp1.centerMassPos); 
//    ab::Vect4 global_L2_CM_pos = (l2->getLocalMatr() * lp2.centerMassPos); 
//    ab::Vect4 global_L3_CM_pos = (l3->getLocalMatr() * lp3.centerMassPos);     
//    ab::Vect4 global_L4_CM_pos = (l4->getLocalMatr() * lp4.centerMassPos); 
//    ab::Vect4 global_L5_CM_pos = (l5->getLocalMatr() * lp5.centerMassPos); 
//    ab::Vect4 l1_angSpeed(0, 0, 1);
//    ab::Vect4 l1_linSpeed = l1_angSpeed.cross((cm_pos - global_L1_CM_pos) * -1);
//    l1->setInitialState(0, 0, l1_linSpeed, l1_angSpeed);
//    ab::Vect4 l2_linSpeed = l1_angSpeed.cross((cm_pos - global_L2_CM_pos) * -1);
//    ab::Real l2_angSpeed = l2_linSpeed.norm() / lp2.centerMassPos.norm();
//    l2->setInitialState(-l2_angSpeed * rel_L2_mass);
//    ab::Vect4 l3_linSpeed = l1_angSpeed.cross((cm_pos - global_L3_CM_pos) * -1);
//    ab::Real l3_angSpeed = l3_linSpeed.norm() / lp3.centerMassPos.norm();
//    l3->setInitialState(l3_angSpeed * rel_L3_mass);
//
//    ab::Vect4 l4_linSpeed = l1_angSpeed.cross((cm_pos - global_L4_CM_pos) * -1);
//    ab::Real l4_angSpeed = l4_linSpeed.norm() / lp4.centerMassPos.norm();
//    l4->setInitialState(l4_angSpeed * rel_L4_mass);
//
//    ab::Vect4 l5_linSpeed = l1_angSpeed.cross((cm_pos - global_L5_CM_pos) * -1);
//    ab::Real l5_angSpeed = l5_linSpeed.norm() / lp5.centerMassPos.norm();
//    l5->setInitialState(l5_angSpeed * rel_L5_mass);

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

    ab::Real allMass = Body_Rt_Mass+/*Hnd_R_Mass+Hnd_L_Mass+Leg_R_Mass+*/Leg_L_Mass;
    ab::Vect4 Body_CM_pos = (/*(Body_Rt_Size_ShR-Hnd_R_Size_a) * Hnd_R_Mass + 
                            (Body_Rt_Size_ShL-Hnd_L_Size_a) * Hnd_L_Mass + 
                            (Body_Rt_Size_HR -Leg_R_Size_a) * Leg_R_Mass + */
                            (Body_Rt_Size_HL -Leg_L_Size_a) * Leg_L_Mass) * 
                            (1 / allMass);    
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
//    ab::JointDOFParams dofsParams[3];    
//    dofsParams[0].axis = ab::Vect4::UnitZ;        
//    dofsParams[1].axis = ab::Vect4::UnitZ;//ab::Vect4::UnitY;
//    dofsParams[2].axis = ab::Vect4::UnitX;
////    dofsParams[0].axis = ab::Vect4::UnitX;        
////    dofsParams[1].axis = ab::Vect4::UnitZ;
//    
//
//    dofsParams[0].springK = 10;   // Z
//    dofsParams[0].dumpK   = 2; // 
//    dofsParams[1].springK = 10;   // Y
//    dofsParams[1].dumpK   = 2; // 
//    dofsParams[2].springK = 10;   // X
//    dofsParams[2].dumpK   = 2; // 
//    ab::Joint* body_left_leg = mb->createJoint(3, dofsParams);
//    dofsParams[0].springK = 10;   // Z
//    dofsParams[0].dumpK   = 2; // 
//    dofsParams[1].springK = 10;   // Y
//    dofsParams[1].dumpK   = 2; // 
//    dofsParams[2].springK = 10;   // X
//    dofsParams[2].dumpK   = 2; // 
//    ab::Joint* body_right_leg = mb->createJoint(3, dofsParams);
//    dofsParams[0].springK = 2;   // Z
//    dofsParams[0].dumpK   = 0.3; // 
//    dofsParams[1].springK = 5;   // Y
//    dofsParams[1].dumpK   = 0.3; // 
//    dofsParams[2].springK = 5;   // X
//    dofsParams[2].dumpK   = 0.5; // 
//    ab::Joint* body_left_arm = mb->createJoint(3, dofsParams);
//    dofsParams[0].springK = 2;   // Z
//    dofsParams[0].dumpK   = 0.3; // 
//    dofsParams[1].springK = 5;   // Y
//    dofsParams[1].dumpK   = 0.3; // 
//    dofsParams[2].springK = 5;   // X
//    dofsParams[2].dumpK   = 0.5; // 
//    ab::Joint* body_right_arm = mb->createJoint(3, dofsParams);
//    mb->setRoot(body);
//    body_left_leg->connect(body, left_leg);

//    body_right_leg->connect(body, right_leg);
//    body_left_arm->connect(body, left_arm);
//    body_right_arm->connect(body, right_arm);

    mb->init();

    _testMb = mb;

    zArmAngController.setDesiredValue(85.0);
}

void Processor::_createTestJointBodies_1(void)
{
    ab::ArtBody* mb = new ab::ArtBody();

    ab::Real Body_Rt_Mass = 29.28;
    
    ab::Real   X_Ang_Correction = 0.0;//-0.014576679271515;
//    ab::Vect4  Body_Rt_Size(0,   0.518,  -0.007);
    ab::Vect4  Body_Rt_Size(0,   0.518,  0.000);
    ab::Vect4  Body_Rt_Inertia(4.258, 1.062, 5.057);
    ab::Real   Body_01_Mass = 30.53;
    ab::Vect4  Body_01_Size(0,  -0.284,  0);
    ab::Vect4  Body_01_Inertia(1.271, 0.288, 1.434);
    ab::Vect4  CM_loc_Rel_Rt = (Body_Rt_Size-Body_01_Size) * ((-1)*Body_01_Mass / (Body_Rt_Mass+Body_01_Mass));
    ab::Vect4  CM_loc_Rel_B01 = (Body_01_Size - Body_Rt_Size) * ((-1)*Body_Rt_Mass / (Body_Rt_Mass+Body_01_Mass));

    ab::Vect4  Body_Rt_IPos =  CM_loc_Rel_Rt;
    ab::Vect4  Body_Rt_IVel(1, 0, 0);
    ab::Vect4  Body_Rt_IVel_Transl =  Body_Rt_IVel.cross(CM_loc_Rel_Rt) * -1;

    ab::Vect4  Body_01_IPos(0, 0, 0);            
    ab::Vect4  Body_01_IVel(0, 0, 0);

    // up body
    ab::Link* up_body;
    ab::LinkParams up_body_lp;
    up_body_lp.mass = Body_Rt_Mass;
    up_body_lp.inertia = Body_Rt_Inertia;
    up_body_lp.lenght        =  2 * Body_Rt_Size.norm();
    up_body_lp.centerMassPos =  Body_Rt_Size * -1;
    up_body_lp.dir           =  ab::Vect4::UnitYNeg;
    up_body_lp.base_ang_vel  =  Body_Rt_IVel;
    up_body_lp.base_lin_vel  =  Body_Rt_IVel_Transl;

    up_body = new ab::Link(up_body_lp);

    // lower body
    ab::Link* lower_body;
    ab::LinkParams lower_body_lp;
    lower_body_lp.mass = Body_01_Mass;
    lower_body_lp.inertia = Body_01_Inertia;
    lower_body_lp.lenght        =  2 * Body_01_Size.norm();
    lower_body_lp.centerMassPos =  Body_01_Size;
    lower_body_lp.modelMatr.translate(ab::Link::VectLocalFromMirtCoord(up_body, Body_Rt_Size * -1));
    lower_body_lp.modelMatr.rotateX(ab::DegToRad(-0.014576679271515));
    lower_body_lp.dir           =  ab::Vect4::UnitYNeg;

    lower_body = new ab::Link(lower_body_lp);

    ab::JointDOFParams dofsParams[3];    
    dofsParams[0].axis = ab::Vect4::UnitX;
    dofsParams[1].axis = ab::Vect4::UnitY; 
    dofsParams[2].axis = ab::Vect4::UnitZ;
    dofsParams[0].springK = 2;   // Z
    dofsParams[0].dumpK   = 0.3; // 
    dofsParams[1].springK = 1;   // Y
    dofsParams[1].dumpK   = 0.3; // 
    dofsParams[2].springK = 2;   // X
    dofsParams[2].dumpK   = 0.3; // 

    ab::Joint* j_u_l = mb->createJoint(3, dofsParams);
    mb->setRoot(up_body);
    j_u_l->connect(up_body, lower_body);

    mb->init();
    mb->getMultiBody()->setGravity(mrt::Vect3(0.0, 0.0, 0.0));
    _testMb = mb;
    _testID = _TestTypeTest1;
}

void Processor::_createTestJointBodies_2(void)
{
    ab::ArtBody* mb = new ab::ArtBody();
    
    // make model
    ab::Link* l1;
    ab::Link* l2;
    ab::LinkParams lp1, lp2;
    
    
    lp1.mass = 1;
    lp1.inertia = ab::Vect4(lp1.mass / 12., 0.01, lp1.mass / 12.);
    lp1.lenght = 1;
    lp1.centerMassPos = ab::Vect4(0, -0.5, 0);
    lp1.dir = ab::Vect4::UnitYNeg;    

    lp2.mass = 1;
    lp2.inertia = ab::Vect4(lp2.mass /12., 0.01,  lp2.mass / 12.);
    lp2.lenght = 1;
    lp2.centerMassPos = ab::Vect4(0, -0.5, 0);
    lp2.dir = ab::Vect4::UnitYNeg;
    
    ab::Vect4  cm_pos(0, 0, 0);
    ab::Vect4  base_l1_ang_vel(1, 0, 0);
    ab::Vect4  base_l1_lin_vel =  base_l1_ang_vel.cross(cm_pos);
    lp1.base_ang_vel = base_l1_ang_vel;
    lp1.base_lin_vel = base_l1_lin_vel;

    l1 = new ab::Link(lp1);
    lp2.modelMatr.translate(ab::Link::VectLocalFromMirtCoord(l1, ab::Vect4(0., 0.5, 0)));   
    l2 = new ab::Link(lp2);

    ab::JointDOFParams dofsParams[3];    
    dofsParams[0].axis = ab::Vect4::UnitX;
    dofsParams[1].axis = ab::Vect4::UnitY; 
    dofsParams[2].axis = ab::Vect4::UnitZ;
    dofsParams[0].springK = 0.0;   // Z
    dofsParams[0].dumpK   = 0; // 
    dofsParams[1].springK = 0;   // Y
    dofsParams[1].dumpK   = 0.; // 
    dofsParams[2].springK = 0;   // X
    dofsParams[2].dumpK   = 0.; // 

    ab::Joint* j_1_2 = mb->createJoint(1, dofsParams);
    mb->setRoot(l1);
    j_1_2->connect(l1, l2);

    mb->init();    
    mb->getMultiBody()->setGravity(mrt::Vect3(0.0, 0.0, 0.0));
    _testMb = mb;
    _testID = _TestTypeTest2;
}

void Processor::_createTestJointBodies_3(void)
{
    ab::ArtBody* mb = new ab::ArtBody();

    // make model
    ab::Link* l1;
    ab::Link* l2;    
    ab::LinkParams lp1, lp2;

    lp1.mass = 1;
    lp1.inertia = ab::Vect4(lp1.mass / 12., 0.01, lp1.mass / 12.);
    lp1.lenght = 1;
    lp1.centerMassPos = ab::Vect4(0, -0.5, 0);
    lp1.dir = ab::Vect4::UnitYNeg;    
    lp1.modelMatr.rotateX(ab::PI / 2.0, false);

    lp2.mass = 1;
    lp2.inertia = ab::Vect4(lp2.mass /12., 0.01,  lp2.mass / 12.);
    lp2.lenght = 1;
    lp2.centerMassPos = ab::Vect4(0, -0.5, 0);
    lp2.dir = ab::Vect4::UnitYNeg;

//    ab::Vect4  cm_pos(0, 0, 0);
//    ab::Vect4  base_l1_ang_vel(1, 0, 0);
//    ab::Vect4  base_l1_lin_vel =  base_l1_ang_vel.cross(cm_pos);
//    lp1.base_ang_vel = base_l1_ang_vel;
//    lp1.base_lin_vel = base_l1_lin_vel;

    l1 = new ab::Link(lp1);
    //lp2.modelMatr.rotateX(ab::PI / 2.0);
    //lp2.modelMatr.translate(ab::Link::VectLocalFromMirtCoord(l1, ab::Vect4(0., -0.5, 0)));       
    lp2.modelMatr.translate(ab::Vect4(0., -1.0, 0));       
    l2 = new ab::Link(lp2);

    ab::JointDOFParams dofsParams[3];    
    dofsParams[0].axis = ab::Vect4::UnitX;
    dofsParams[1].axis = ab::Vect4::UnitY; 
    dofsParams[2].axis = ab::Vect4::UnitZ;
    dofsParams[0].springK = 0.0;   // Z
    dofsParams[0].dumpK   = 0; // 
    dofsParams[1].springK = 0;   // Y
    dofsParams[1].dumpK   = 0.; // 
    dofsParams[2].springK = 0;   // X
    dofsParams[2].dumpK   = 0.; // 

    ab::Joint* root_joint = mb->createJoint(1, dofsParams);    
    ab::Joint* j_1_2 = mb->createJoint(1, dofsParams);
    mb->setRoot(l1, root_joint);
    j_1_2->connect(l1, l2);

    mb->init();    
    mb->getMultiBody()->setGravity(mrt::Vect3(0.0, -9.81, 0.0));
    _testMb = mb;
    _testID = _TestTypeTest3;
}


void Processor::_createChains(void)
{
    ab::Matr4 topMatr;

    _createOneChain(topMatr);
    topMatr.translate(ab::Vect4(2.0, 0.0, 0.0));
    _createOneChain(topMatr);
    topMatr.translate(ab::Vect4(0.0, 0.0, -2.0));
    _createOneChain(topMatr);
    topMatr.translate(ab::Vect4(-2.0, 0.0, 0.0));
    _createOneChain(topMatr);

    return;
}                            

void Processor::_createOneChain(const ab::Matr4& topMatr)
{
//    obj::MultiBody * mb;// = _mbManager.createMultiBody();
//
//    ab::Matr4 modelMatr = topMatr;
//    obj::LinkParams lp;
//    lp.lenght        = 0.2;
//    lp.mass          = 1.0;
//    lp.inertia       = ab::Vect4(lp.lenght * lp.lenght, 0, lp.lenght * lp.lenght) * (lp.mass / 12.0);
//    lp.centerMassPos = ab::Vect4(0.0, -lp.lenght / 2.0, 0.0);
//    lp.modelMatr     = topMatr;
//    lp.modelMatr.rotateZ(ab::PI / 2.0, false);
//    lp.dir           = ab::Vect4::UnitYNeg;
//
//    obj::Link * parent = mb->getRoot();
//    for (int i = 0; i < 10; i++) {
//        //modelMatr.rotateY(ab::PI / 2.0, true);
//        //if (i == 0) {
//        //    modelMatr.rotateZ(ab::PI / 2.0, false);
//        //}
//        //obj::Link* link2 = createNullJoint(modelMatr);
//        //mb->addChild(parent, link2);
//        //parent = link2;
//
//        //modelMatr = ab::Matr4::Identity;
//        //if (i == 0) {
//        //    modelMatr.rotateZ(-ab::PI / 2.0, false);
//        //} else {
//        //    modelMatr.rotateY(ab::PI / 2.0, true);
//        //}
//        //obj::Link* link3 = createNullJoint(modelMatr);
//        //mb->addChild(parent, link3);
//        //parent = link3;
//
//        obj::Link* link1 = new obj::Link(lp);
//        mb->addChild(parent, link1);
//        parent = link1;
//
//        modelMatr    = ab::Matr4::Identity;
//        modelMatr.translate(ab::Vect4(0.0, -lp.lenght, 0.0));
//        lp.modelMatr = modelMatr;
//    }
//
//    mb->init();
}

//
// Controller class
//

Controller::Controller(ab::Real intergationStep, ab::Real acc, ab::Real velBound) :
_integrationStep(intergationStep),
_desiredPos(0),
_velBound(velBound),
_curPos(0),
_curVel(0),
_curAcc(acc)
{    
}

void Controller::processStep()
{
    if (ab::isZero(_curPos - _desiredPos)) {
        // nothing to do
        return;
    }

    ab::Real sign = 1;
    if (_curPos > _desiredPos) {
        sign = -1;
    }

    // integrate acc -> vel
    _curVel = _integrate(_curVel, _curAcc, _integrationStep, 1, _velBound);
    // intergate vel -> pos
    _curPos = _integrate(_curPos, _curVel, _integrationStep, sign, _desiredPos);
}

void Controller::setDesiredValue(ab::Real value)
{
    _desiredPos = value;
    _curVel     = 0;
}

ab::Real Controller::_integrate(ab::Real curValue, ab::Real deltaValue, ab::Real deltaT, ab::Real sign, ab::Real bound)
{
    ab::Real newValue = curValue;

    newValue += deltaValue * deltaT * sign;

    if (sign > 0) {
        newValue = (newValue < bound) ? newValue : bound;
    } else  {
        newValue = (newValue > bound) ? newValue : bound;
    }

    return newValue;
}