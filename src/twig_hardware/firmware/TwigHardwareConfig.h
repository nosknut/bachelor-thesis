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
  uint16_t maxShoulderCurrent = 0;
  uint16_t maxWristCurrent = 0;
  uint16_t maxGripperCurrent = 0;
  int16_t maxCurrentDuration = 0;
  int16_t maxCurrentCooldownDuration = 0;
  int16_t encoderMinMagnitude = 0;
};

#endif