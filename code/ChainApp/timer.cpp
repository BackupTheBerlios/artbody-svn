//////////////////////////////////////////////////////////////////////////
// timer.cpp 
//////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <Windows.h>
#include <gl/glut.h>

#include "ab_types.h"
#include "timer.h"


using namespace tmr;

Timer::Timer(void)
{
   return;
}

Timer::~Timer()
{
   ;
}

bool Timer::addHdl(TimerHdl * pHdl)
{
   if (sys::HdlCollection<TimerHdl>::addHdl(pHdl)) {
      pHdl->_timer = this;
      return true;
   }
   return false;
}

bool Timer::removeHdl(TimerHdl * pHdl)
{
   if (sys::HdlCollection<TimerHdl>::removeHdl(pHdl)) {
      pHdl->_timer = NULL;
      return true;
   }
   return false;
}

void Timer::processTimer(void)
{
   unsigned int i;
   _counter._addFrame();
   for (i = 0; i < _hdlArray.size(); i++) {
      _hdlArray[i]->onTimer(_counter.getFPS());
   }
   return;
}

ab::Real Timer::getCurTime(void) const
{
    return _counter._getCurTime();
}

//////////////////////////////////////////////////////////////////////////
// Counter()
//////////////////////////////////////////////////////////////////////////
Counter::Counter() :
_perfFreq(0)
{
   // init array by default fake falues
   for (int i = 0; i < _N_VALS_TO_STORE; i++) {
      _elapsedArr[i] = 0.01f;
   }   
   _lastIdx = 1;

   // init high-resolution performance counter
   {        
       LARGE_INTEGER freq;
       if ( QueryPerformanceFrequency(&freq) ) {
           _perfFreq = freq.LowPart;
       } else {
           _perfFreq = 0;
       }
   }

   _lastTime = _getCurTime();

}

ab::Real Counter::getFrameElapsed(void)
{
   int lastIndex;

   lastIndex = _GetLastIndex();

   return _elapsedArr[lastIndex];
}

ab::Real Counter::getFPS(void)
{
   ab::Real fps = 0.f, allTime = 0.f;
   unsigned int i;
   for (i = 0; i < _N_VALS_TO_STORE; i++) {
      allTime += _elapsedArr[i];
   }
   return allTime / _N_VALS_TO_STORE;
}

void Counter::_addFrame(void)
{
   ab::Real newTime = _getCurTime();
   ab::Real elapsed = newTime - _lastTime;
   
   int lastIndex = _GetLastIndex();
   _elapsedArr[lastIndex] = elapsed;   
   _lastIdx = (_lastIdx < _N_VALS_TO_STORE - 1) ? (_lastIdx + 1) : 0;   
   _lastTime = newTime;
}

ab::Real Counter::_getCurTime(void) const
{
   if (0/*_perfFreq*/) {
       LARGE_INTEGER curTiks;
       QueryPerformanceCounter((LARGE_INTEGER*)&curTiks);
       return (ab::Real)(curTiks.LowPart / _perfFreq);
   } else {
        ab::DWord timeDword = GetTickCount();
        return timeDword / 1000.f;
   }
}

int Counter::_GetLastIndex(void)
{
   return (_lastIdx > 0) ? (_lastIdx - 1) : (_N_VALS_TO_STORE - 1);
}

