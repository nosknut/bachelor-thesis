#ifndef ENCODER_h
#define ENCODER_h

#include <Arduino.h>
#include <SoftI2C.h>
#include "Lowpass.h"
#include "AS5600.h"
#include "AngularVelocityTracker.h"
#include "FirmwareConfig.h"
#include "EncoderValues.h"

class Encoder
{
  const int sdaPin;
  const int sclPin;

  AngularVelocityTracker velocityTracker;

public:
  int minMagnitude;
  const String name;
  uint16_t rawAngle = 0;
  EncoderValues values;
  LowPass<POSITION_LOWPASS_FILTER_ORDER> filter =
      LowPass<POSITION_LOWPASS_FILTER_ORDER>(POSITION_LOWPASS_FILTER_CUTOFF, 1e3, true);

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
    // Instantiating both the bus and the encoder for each execution to save
    // memory.
    SoftI2C i2cBus = SoftI2C(sdaPin, sclPin);
    i2cBus.begin();
    AS5600 encoder = AS5600(&i2cBus);

    values.magnitude = encoder.readMagnitude();
    if (values.magnitude > minMagnitude) {
      rawAngle = encoder.readAngle();
      filter.update(rawAngle);
      values.angle = filter.value;
    } else {
#ifdef DEBUG_ENCODERS
      Serial.println("ERROR: " + name + " encoder magnet is too weak");
#endif

      i2cBus.end();
      return false;
    }

    i2cBus.end();
    return true;
  }

  void updateVelocity()
  {
    values.velocity = velocityTracker.update(values.angle);
  }

public:
  void begin()
  {
  }

  void end()
  {
  }

  bool update()
  {
    if (updateAngle()) {
      updateVelocity();
      return true;
    }
    return false;
  }
};

#endif
