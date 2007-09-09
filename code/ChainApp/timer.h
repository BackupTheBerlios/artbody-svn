#ifndef _TIMER_H_
#define _TIMER_H_

#include "hdl_collect.h"

namespace tmr {

class Timer;
class TimerHdl;
class Counter;

class Counter {
   friend class Timer;
public:
   Counter();

   ab::Real getFrameElapsed(void);
   ab::Real getFPS         (void);
private:
   void      _addFrame  (void);
   ab::Real  _getCurTime(void) const;

   int       _GetLastIndex(void);

   static const int _N_VALS_TO_STORE = 50;

   // pool
   ab::Real    _elapsedArr[_N_VALS_TO_STORE];
   int         _lastIdx;

   ab::Real    _lastTime;
   ab::DWord   _perfFreq;
};

class TimerHdl {
friend class Timer;
private:
   virtual void onTimer(ab::Real elapsed) = 0;
protected:
   const Timer* _timer;
};

class Timer : public sys::HdlCollection<TimerHdl> {
public:
   explicit Timer(void);
   virtual ~Timer();

   virtual void  processTimer(void);

   bool  addHdl   (TimerHdl* pHdl);
   bool  removeHdl(TimerHdl* pHdl);
    
   ab::Real getCurTime() const;
protected:
   Counter _counter;
};

} // namespace tmr

#endif //_TIMER_H_
