#ifndef ANGULAR_SPEED_TRACKER_h
#define ANGULAR_SPEED_TRACKER_h

#include <Arduino.h>
#include "AS5600.h"
#include "RunningMedian.h"

class AngularVelocityTracker
{

  unsigned long prevTime = micros();
  double prevAngle = 0;

public:
  double speed = 0;
  
  // Based on code in AS5600 lib
  double update(double angle)
  {
    unsigned long now = micros();
    unsigned long deltaT = now - prevTime;

    double deltaA = angle - prevAngle;

    //  assumption is that there is no more than 180° rotation
    //  between two consecutive measurements.
    //  => at least two measurements per rotation (preferred 4).
    if (deltaA > 2048)
    {
      deltaA -= 4096;
    }

    if (deltaA < -2048)
    {
      deltaA += 4096;
    }

    speed = ((deltaA * 1e6) / deltaT) * AS5600_RAW_TO_DEGREES;

    prevAngle = angle;
    prevTime = now;

    return speed;
  }
};

#endif