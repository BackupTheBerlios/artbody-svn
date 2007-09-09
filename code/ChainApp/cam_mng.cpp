//////////////////////////////////////////////////////////////////////////
// Processor.cpp
//////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <gl/glut.h>

#include "render.h"
#include "timer.h"
#include "processor.h"
#include "camera.h"

#include "ab_adapter.h"
#include "ab_types.h"
#include "ab_vect4.h"
#include "ab_matr4.h"

using namespace app;

CameraManager::CameraManager() :
_pCamera(NULL),
_moveType(_MOVE_TYPE_NONE)
{
   ;
}

void CameraManager::setCamera(rnd::Camera& camera)
{
   _pCamera = &camera;
}

bool CameraManager::setCameraWindow(int w, int h)
{
   if (_pCamera == NULL) {
      return false;
   }

   rnd::CameraParams params;

   _pCamera->getParams(params);

   params.xView = (ab::Real)w;
   params.yView = (ab::Real)h;
   params.aspect = params.xView / params.yView;

   _pCamera->setParams(params);

   return true;
}

bool CameraManager::handleCameraInput(const inp::InputParams& params)
{
   if (_handleKbdCommands(params)) {
      return true;
   }

   switch (_moveType) {
      case _MOVE_TYPE_ROT:
         if (params.button == inp::MouseBtnLeft && params.msBtnState == inp::BtnUp) {
            _moveType = _MOVE_TYPE_NONE;            
         } else {
            _rotateCamera(params);
         }
   	   break;
      case _MOVE_TYPE_XY:
         if (params.button == inp::MouseBtnMiddle && params.msBtnState == inp::BtnUp) {
            _moveType = _MOVE_TYPE_NONE;            
         } else {
            _moveCameraXY(params);
         }
         break;
      case _MOVE_TYPE_Z:
         if (params.button == inp::MouseBtnRight && params.msBtnState == inp::BtnUp) {
            _moveType = _MOVE_TYPE_NONE;            
         } else {
            _moveCameraZ(params);
         }
         break;
      case _MOVE_TYPE_NONE:
         // camera move only if 'Alt' key is pressed
         if (params.msBtnState == inp::BtnDown && (params.sysKeyState & inp::ALT_KEY_PRESS)) {
            if (params.button == inp::MouseBtnLeft)         _moveType = _MOVE_TYPE_ROT;
            else if (params.button == inp::MouseBtnMiddle)  _moveType = _MOVE_TYPE_XY;
            else if (params.button == inp::MouseBtnRight)   _moveType = _MOVE_TYPE_Z;
         }
         break;
   }

   _prevInputParams = params;   
   
   return true;
}

bool CameraManager::_handleKbdCommands(const inp::InputParams& inpParams)
{
   ab::Matr4 matrC2W;

   switch (inpParams.sym) {
      case 'F':
      case 'f':
         matrC2W.setAxisZ(ab::Vect4::UnitZNeg);
         _pCamera->setMatrC2W(matrC2W);
         return true;
      default:
         break;
   }   
   return false;
}

void CameraManager::_rotateCamera(const inp::InputParams& inpParams)
{  
   // one pixel = 0.005 radian
   ab::Real diff_x = (inpParams.x - _prevInputParams.x) / 200.f;
   ab::Real diff_y = (_prevInputParams.y - inpParams.y) / 200.f;

   ab::Matr4 matrC2W = _pCamera->getMatrC2W();
//   ab::Vect4 pointOfView = _GetPointOfView();
//   ab::Vect4 curPos = matrC2W.getTranslate();
//   ab::Real moveLen = (pointOfView - curPos).norm();
//   
//   matrC2W.translateLocal(ab::Axis_Z, moveLen);
//   matrC2W.rotateX(diff_y);
//   matrC2W.rotateY(diff_x);
//   matrC2W.translateLocal(ab::Axis_Z, -moveLen);

   // test - rotation around Zero
   ab::Matr4 matrW2C(matrC2W);
   matrW2C.invert();
   matrC2W.transform(matrW2C, false);
   matrC2W.rotateX(diff_y, false);
   matrC2W.rotateY(diff_x, false);
   matrW2C.invert();
   matrC2W.transform(matrW2C, false);

   _pCamera->setMatrC2W(matrC2W);

   return;
}

void CameraManager::_moveCameraXY(const inp::InputParams& inpParams)
{
   // one pixel = 0.01 u.e. :)
   ab::Real diff_x = (inpParams.x - _prevInputParams.x) / 100.f;
   ab::Real diff_y = (inpParams.y - _prevInputParams.y) / 100.f;
   
   ab::Matr4 matrC2W = _pCamera->getMatrC2W();

   ab::Vect4 axisX = matrC2W.getAxisX();   
   axisX.scale(diff_x);
   matrC2W.translate(axisX * -1);
   
   ab::Vect4 axisY = matrC2W.getAxisY();   
   axisY.scale(diff_y);
   matrC2W.translate(axisY);
   
   _pCamera->setMatrC2W(matrC2W);

   return;
}

void CameraManager::_moveCameraZ(const inp::InputParams& inpParams)
{
   // one pixel = 0.01 u.e. :)
   ab::Real diff_x = (inpParams.x - _prevInputParams.x) / 100.f;
   ab::Matr4 matrC2W = _pCamera->getMatrC2W();
   ab::Vect4 axisZ = matrC2W.getAxisZ();
   axisZ.scale(diff_x);
   matrC2W.translate(axisZ);
   _pCamera->setMatrC2W(matrC2W);
                  
   return;
}

ab::Vect4 CameraManager::_GetPointOfView(void) const
{
   const ab::Matr4& matrC2W = _pCamera->getMatrC2W();
   // return matrC2W.getTranslate() + (matrC2W.getAxisZ() * 10);
   return ab::Vect4::Zero;
}



