 //////////////////////////////////////////////////////////////////////////
// inputer.cpp 
//////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <Windows.h>
#include <gl/glut.h>

#include "inputer.h"

using namespace inp;

Inputer::Inputer(void)
{
   _clearState();
   return;
}

Inputer::~Inputer()
{
   ;
}

bool Inputer::addHdl(InputerHdl * pHdl)
{
   if (sys::HdlCollection<InputerHdl>::addHdl(pHdl)) {
      pHdl->_inputer = this;
      return true;
   }
   return false;
}

bool Inputer::removeHdl(InputerHdl * pHdl)
{
   if (sys::HdlCollection<InputerHdl>::removeHdl(pHdl)) {
      pHdl->_inputer = NULL;
      return true;
   }
   return false;
}

void Inputer::processInput(const InputParams& params)
{
   unsigned int i;   
   _curParams = params;
   for (i = 0; i < _hdlArray.size(); i++) {
      _hdlArray[i]->onInput(params);
   }   
   _clearState();
   return;
}

void Inputer::MapGLInputToApp(const unsigned char* c, const int* msButton, const int* msBtnState, 
                              const int* sysBtnState, const int* x, const int* y, 
                              inp::InputParams& params)
{
    if (c) {
        params.sym = *c;
    }

    if (msButton) {
        switch (*msButton) {
         case GLUT_LEFT_BUTTON:
             params.button = inp::MouseBtnLeft;
             break;
         case GLUT_MIDDLE_BUTTON:
             params.button = inp::MouseBtnMiddle;
             break;
         case GLUT_RIGHT_BUTTON:
             params.button = inp::MouseBtnRight;
             break;
        }
    }

    if (msBtnState) {
        switch (*msBtnState) {
         case GLUT_UP:
             params.msBtnState = inp::BtnUp;
             break;
         case GLUT_DOWN:
             params.msBtnState = inp::BtnDown;
             break;
        }
    }

#define CHECK_AND_SET(srcState, srcStateVal, dstState, dstStateVal) \
    dstState |= (srcState & srcStateVal) ? dstStateVal : 0
    if (sysBtnState) {
        CHECK_AND_SET(*sysBtnState, GLUT_ACTIVE_SHIFT, params.sysKeyState, inp::SHIFT_KEY_PRESS);      
        CHECK_AND_SET(*sysBtnState, GLUT_ACTIVE_CTRL,  params.sysKeyState, inp::CTRL_KEY_PRESS);      
        CHECK_AND_SET(*sysBtnState, GLUT_ACTIVE_ALT,   params.sysKeyState, inp::ALT_KEY_PRESS);      
    }
#undef CHECK_AND_SET

    if (x) {
        params.x = *x;
    }

    if (y) {
        params.y = *y;
    }

    return;
}



//void Inputer::onKeyboardInput(unsigned char c, int x, int y)
//{
//   _curParams.sym 
//   _sym = c;
//   _x = x;
//   _y = y;
//   processInput();
//   return;
//}
//
//void Inputer::onMouseInput(int button, int state, int x, int y)
//{
//   _button   = button;
//   _btnState = state;
//   _x = x;
//   _y = y;
//   processInput();
//   return;
//}


//
// private 
//

void Inputer::_clearState()
{
   InputParams defParams;
   
   _curParams.sym = defParams.sym;
   _curParams.button     = defParams.button;
   _curParams.msBtnState = defParams.msBtnState;
   _curParams.sysKeyState = defParams.sysKeyState;
   _curParams.x = defParams.x;
   _curParams.y = defParams.y;

   return;
}

//////////////////////////////////////////////////////////////////////////
// InputParams
//////////////////////////////////////////////////////////////////////////
InputParams::InputParams()
{
   sym         = 0;
   button      = MouseBtnUndef;
   msBtnState  = BtnUndef;
   sysKeyState = 0;
   x           = 0;
   y           = 0;
}
