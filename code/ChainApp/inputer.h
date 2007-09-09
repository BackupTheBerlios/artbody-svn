#ifndef _INPUTER_H_
#define _INPUTER_H_

#include "hdl_collect.h"
#include "ab_adapter.h"

namespace inp {

class Inputer;
class InputerHdl;

enum MouseBtn {
   MouseBtnLeft,
   MouseBtnMiddle,
   MouseBtnRight,
   MouseBtnUndef
};

enum BtnState {
   BtnDown,
   BtnUp,
   BtnUndef
};

const ab::DWord SHIFT_KEY_PRESS = 0x00000001;
const ab::DWord CTRL_KEY_PRESS  = 0x00000002;
const ab::DWord ALT_KEY_PRESS   = 0x00000004;

enum SysKeyState {
};

struct InputParams {
   InputParams();
   unsigned char sym;
   MouseBtn      button;
   BtnState      msBtnState; 
   ab::DWord     sysKeyState;
   int           x;
   int           y;
};

class InputerHdl {
friend class Inputer;
private:
   virtual void onInput(const InputParams& params) = 0;
protected:
   Inputer* _inputer;
};

class Inputer : public sys::HdlCollection<InputerHdl> {
public:
   explicit Inputer(void);
   virtual ~Inputer();

//   virtual void  onKeyboardInput(unsigned char c, int x, int y);
//   virtual void  onMouseInput   (int button, int state, int x, int y);

   virtual void  processInput   (const InputParams& params);

   bool  addHdl   (InputerHdl* pHdl);
   bool  removeHdl(InputerHdl* pHdl);

   const InputParams& getCurParams(void) { return _curParams; }

   // helper
   static void MapGLInputToApp(const unsigned char* c, const int* msButton, const int* msBtnState,
                               const int* sysBtnState, const int* x, const int* y, 
                               inp::InputParams& params);

protected:
   InputParams _curParams;
private:
   void _clearState(void);
};

} // namespace inp

#endif //_INPUTER_H_
