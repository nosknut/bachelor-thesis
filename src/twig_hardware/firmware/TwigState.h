#ifndef TWIG_STATE_h
#define TWIG_STATE_h

// Attribute packed is used to ensure the struct has the same
// shape across different architectures.
// This allows it to be sent through the i2c bus.
// https://stackoverflow.com/questions/21092415/force-c-structure-to-pack-tightly
// https://www.geeksforgeeks.org/how-to-avoid-structure-padding-in-c/
struct __attribute__((packed)) TwigState
{
  int16_t integrityCheck = 85;
  int16_t sessionId = 0;
  int16_t wristPosition = 0;
  int16_t gripperPosition = 0;
  int16_t shoulderPosition = 0;
  float wristVelocity = 0;
  float gripperVelocity = 0;
  float shoulderVelocity = 0;
  uint8_t wristEncoderMagnitude = 0;
  uint8_t gripperEncoderMagnitude = 0;
  uint8_t shoulderEncoderMagnitude = 0;
  uint8_t wristCurrent = 0;
  uint8_t gripperCurrent = 0;
  uint8_t shoulderCurrent = 0;
  uint8_t wristVoltage = 0;
  uint8_t shoulderVoltage = 0;
  bool shoulderServoPowered = false;
  bool wristServoPowered = false;
  bool gripperServoPowered = false;
};

#endif