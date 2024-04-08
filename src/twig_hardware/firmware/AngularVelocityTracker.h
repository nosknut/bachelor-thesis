#ifndef ANGULAR_SPEED_TRACKER_h
#define ANGULAR_SPEED_TRACKER_h

#include <Arduino.h>
#include "AS5600.h"
#include "RunningMedian.h"

class AngularVelocityTracker
{

  RunningMedian angleFilter = RunningMedian(2);
  RunningMedian speedMedianFilter = RunningMedian(3);
  RunningMedian speedFilter = RunningMedian(3);

  unsigned long prevTime = micros();
  double prevAngle = 0;

public:
  // Based on code in AS5600 lib
  double update(double angle)
  {
    unsigned long now = micros();
    unsigned long deltaT = now - prevTime;

    angleFilter.add(angle);
    double deltaA = angleFilter.getMedian() - prevAngle;

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

    speedMedianFilter.add(((deltaA * 1e6) / deltaT) * AS5600_RAW_TO_DEGREES);
    speedFilter.add(speedMedianFilter.getMedian());

    prevAngle = angleFilter.getMedian();
    prevTime = now;

    return speedFilter.getAverage();
  }
};

#endif