#ifndef _CHAIN_APP_H_
#define _CHAIN_APP_H_

// forvard
namespace rnd {
   class Renderer;
}

namespace tmr {
   class Timer;
}

namespace inp {
   class Inputer;
}

namespace app {

class Processor;

class Application {
public:
   explicit Application();
   virtual ~Application();

   bool Init    (int argc, char * argv[]);
   void Term    (void);
   int  MainLoop(void);
private:
   bool  _InitSystem  (int argc, char * argv[]);

   void  _TermSystem  (void);

   static void __cdecl _DisplayFunc (void);
   static void __cdecl _IdleFunc    (void);
   static void __cdecl _ReshapeFunc (int w, int h);
   static void __cdecl _KeyboardFunc(unsigned char c, int x, int y);
   static void __cdecl _MouseFunc   (int button, int state, int x, int y);
   static void __cdecl _MotionFunc  (int x, int y);

   static const int _wSize = 400;
   static const int _hSize = 400; 

   rnd::Renderer* _renderer;
   tmr::Timer*    _timer;
   inp::Inputer*  _inputer;

   Processor*     _processor;

   int            _window;
};


extern Application App;

}

#endif //_CHAIN_APP_H_
