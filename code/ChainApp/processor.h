#ifndef _PROCESSOR_H_
#define _PROCESSOR_H_

#include <vector>
#include <list>
#include "ab_types.h"
#include "render.h"
#include "timer.h"
#include "inputer.h"

namespace ab {
class MultiBody;
class Link;
struct LinkParams;
}

namespace res {
//class ResultsGrabber;
}

namespace ant {
struct ANT_DESK;
}

namespace ab {
class  ArtBody;
class  RagDollModel;
}

namespace app {

class Processor;
class CameraManager;
class MultiBodyManager;

class CameraManager
{
public:
   CameraManager      (void);
   void     setCamera(rnd::Camera& camera);

   bool     setCameraWindow(int w, int h);

   bool     handleCameraInput(const inp::InputParams& params);
private:
   bool _handleKbdCommands(const inp::InputParams& params);

   void _rotateCamera(const inp::InputParams& inpParams);
   void _moveCameraXY(const inp::InputParams& inpParams);
   void _moveCameraZ (const inp::InputParams& inpParams);

   ab::Vect4 _GetPointOfView(void) const;

   rnd::Camera*     _pCamera;
   inp::InputParams _prevInputParams;
   enum _MoveType {
      _MOVE_TYPE_ROT,
      _MOVE_TYPE_XY,
      _MOVE_TYPE_Z,
      _MOVE_TYPE_NONE
   };
   _MoveType _moveType;

};

class Processor : public rnd::RenderHdl,
                  public tmr::TimerHdl,
                  public inp::InputerHdl 
{
public:
   explicit Processor();
   virtual ~Processor();

   void onRender(void);
   void onTimer (ab::Real elapsed);
   void onInput (const inp::InputParams& params);
   
   inline CameraManager&    getCameraMng(void) { return _cameraMng; }
//   inline MultiBodyManager& getMBManager(void) { return _mbManager; }
private:
   bool _handleSwitchInputStage(const inp::InputParams& params);

   void _createTestBodies       (void);
   void _createTestJointBodies  (void);
   void _createTestJointBodies_1(void);
   void _createTestJointBodies_2(void);
   void _createTestJointBodies_3(void);
   void _createChains           (void);
   void _createOneChain         (const ab::Matr4& topMatr);

   enum _InputStage {
      _INPUT_NOT_PROCESS,
      _INPUT_CAMERA,
      _INPUT_ACT,
      _INPUT_IDLE,
      _INPUT_MAX
   };
   _InputStage _inputStage;
   bool        _isIdle;

   enum _TestType {
      _TestTypeNone,     
      _TestTypeRagDoll,
      _TestTypeTest1,
      _TestTypeTest2,
      _TestTypeTest3
   };

   _TestType         _testID;

   CameraManager     _cameraMng;
   ab::RagDollModel* _pRagDoll;

    // test data
   ab::ArtBody*   _testMb;
};


} // namespace app

#endif //_PROCESSOR_H_
