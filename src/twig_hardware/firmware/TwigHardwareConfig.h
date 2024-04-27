#ifndef TWIG_HARDWARE_CONFIG_h
#define TWIG_HARDWARE_CONFIG_h

// Attribute packed is used to ensure the struct has the same
// shape across different architectures.
// This allows it to be sent through the i2c bus.
// https://stackoverflow.com/questions/21092415/force-c-structure-to-pack-tightly
// https://www.geeksforgeeks.org/how-to-avoid-structure-padding-in-c/
struct __attribute__((packed)) TwigHardwareConfig
{
  int16_t connectionTimeout = 0;
  int16_t wristMaxCurrent = 0;
  int16_t gripperMaxCurrent = 0;
  int16_t shoulderMaxCurrent = 0;
  int16_t wristMaxCurrentDuration = 0;
  int16_t gripperMaxCurrentDuration = 0;
  int16_t shoulderMaxCurrentDuration = 0;
  int16_t shoulderMaxCurrentCooldownDuration = 0;
  int16_t wristMaxCurrentCooldownDuration = 0;
  int16_t gripperMaxCurrentCooldownDuration = 0;
  int16_t wristEncoderMinMagnitude = 0;
  int16_t gripperEncoderMinMagnitude = 0;
  int16_t shoulderEncoderMinMagnitude = 0;
};

#endif