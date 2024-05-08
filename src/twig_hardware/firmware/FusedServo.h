#ifndef FUSED_SERVO_h
#define FUSED_SERVO_h

#include <Arduino.h>
#include <Servo.h>
#include "Fuse.h"
#include "FirmwareConfig.h"

class FusedServo
{
public:
  const int servoPin;
  const int relayPin;
  Servo servo;
  Fuse fuse;

  int16_t speed = 0;
  bool relayState = false;

  FusedServo(int servoPin, int relayPin)
  : servoPin(servoPin), relayPin(relayPin)
  {
  }

  void begin()
  {
    pinMode(relayPin, OUTPUT);
    servo.attach(servoPin);
  }

  void update(uint16_t current, bool relayCommand, int16_t commandSpeed)
  {
    fuse.update(current);

    // If the fuse is tripped, keep the servo stationary and the relay off for the cooldown time
    if (fuse.tripped) {
      speed = 0;
      relayState = false;
    } else {
      speed = commandSpeed;
      relayState = relayCommand;
    }

    write();
  }

  void write()
  {
    servo.writeMicroseconds(SERVO_STATIONARY_SIGNAL + speed);
    digitalWrite(relayPin, relayState ? LOW : HIGH);
  }

};
#endif
