#ifndef FUSE_h
#define FUSE_h

#include <Arduino.h>
#include "Timer.h"

class Fuse
{
public:
  Timer maxCurrentTimer;
  Timer cooldownTimer;

  bool tripped = false;

  uint16_t maxCurrentDuration = 0;
  uint16_t maxCurrentCooldownDuration = 0;
  uint16_t maxCurrent = 0;

  void update(uint16_t current)
  {

    // Keep the fuse timer at zero as long as the current remains within the max limit
    if (current <= maxCurrent) {
      maxCurrentTimer.reset();
    }

    // Trip the fuse and its cooldown if current exceeds max limit for too long
    if (!tripped && maxCurrentTimer.isFinished(maxCurrentDuration)) {
      tripped = true;
      cooldownTimer.reset();
    }

    // Untrip the fuse after the cooldown time expires
    if (tripped && cooldownTimer.isFinished(maxCurrentCooldownDuration)) {
      tripped = false;
    }
  }

};
#endif
