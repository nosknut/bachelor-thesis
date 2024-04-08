#ifndef ENCODER_h
#define ENCODER_h

#include <Arduino.h>
#include <SoftI2C.h>
#include "AS5600.h"
#include "AngularVelocityTracker.h"
#include "EncoderValues.h"

class Encoder
{
  const int sdaPin;
  const int sclPin;
  const int minMagnitude;

  SoftI2C i2cBus = SoftI2C(sdaPin, sclPin);
  AS5600 encoder = AS5600(&i2cBus);
  AngularVelocityTracker velocityTracker;

public:
  const String name;
  EncoderValues values;

  // constructor:
  Encoder(int sdaPin, int sclPin, int minMagnitude, String name)
      : sdaPin(sdaPin), sclPin(sclPin), minMagnitude(minMagnitude), name(name)
  {
  }

  ~Encoder()
  {
    end();
  }

private:
  bool updateAngle()
  {
    if (encoder.readMagnitude() > minMagnitude)
    {
      values.angle = encoder.readAngle();
    }
    else
    {
#ifdef DEBUG_ENCODERS
      Serial.println("ERROR: " + name + " encoder magnet is too weak");
#endif
      return false;
    }

    return true;
  }

  void updateVelocity()
  {
    values.velocity = velocityTracker.update(values.angle);
  }

public:
  void begin()
  {
    i2cBus.begin();
  }

  void end()
  {
    i2cBus.end();
  }

  bool update()
  {
    if (updateAngle())
    {
      updateVelocity();
      return true;
    }
    return false;
  }
};

#endif