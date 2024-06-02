#ifndef ENCODER_h
#define ENCODER_h

#include <Arduino.h>
#include "Lowpass.h"
#include "AS5600.h"
#include "AngularVelocityTracker.h"
#include "FirmwareConfig.h"
#include "EncoderValues.h"
#include "I2cMultiplexer.h"

class Encoder
{
  AngularVelocityTracker velocityTracker;

public:
  int minMagnitude;
  const String name;
  uint16_t rawAngle = 0;
  EncoderValues values;
  int multiplexerChannel;
  I2cMultiplexer &i2cMultiplexer;
  AS5600 encoder;
  LowPass<POSITION_LOWPASS_FILTER_ORDER> filter =
      LowPass<POSITION_LOWPASS_FILTER_ORDER>(POSITION_LOWPASS_FILTER_CUTOFF, 1e3, true);

  // constructor:
  Encoder(I2cMultiplexer &i2cMultiplexer, int multiplexerChannel, int minMagnitude, String name)
      : i2cMultiplexer(i2cMultiplexer), multiplexerChannel(multiplexerChannel),
        minMagnitude(minMagnitude), name(name), encoder(AS5600(&i2cMultiplexer.bus))
  {
  }

private:
  bool updateAngle()
  {
    if (!i2cMultiplexer.selectChannel(multiplexerChannel))
      return false;
    if (!i2cMultiplexer.deviceExists(encoder.getAddress())) {
#ifdef DEBUG_ENCODERS
      Serial.println("ERROR: " + name + " encoder not found");
#endif
      return false;
    }
    
    values.magnitude = encoder.readMagnitude();
    if (values.magnitude > minMagnitude) {
      rawAngle = encoder.readAngle();
      filter.update(rawAngle);
      values.angle = filter.value;
    } else {
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
