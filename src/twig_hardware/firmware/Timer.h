#ifndef TIMER_h
#define TIMER_h

#include <Arduino.h>

class Timer
{
  unsigned long startTime = 0;

public:
  void reset()
  {
    startTime = millis();
  }

  unsigned long elapsed()
  {
    return millis() - startTime;
  }

  bool isFinished(unsigned long duration)
  {
    return elapsed() >= duration;
  }
};

#endif
