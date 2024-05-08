#ifndef TWIG_COMMAND_h
#define TWIG_COMMAND_h

#include "TwigHardwareConfig.h"

// Attribute packed is used to ensure the struct has the same
// shape across different architectures.
// This allows it to be sent through the i2c bus.
// https://stackoverflow.com/questions/21092415/force-c-structure-to-pack-tightly
// https://www.geeksforgeeks.org/how-to-avoid-structure-padding-in-c/
struct __attribute__((packed)) TwigCommand
{
  int16_t sessionId = 0;
  int16_t wrist = 0;
  int16_t gripper = 0;
  int16_t shoulder = 0;
  bool shoulderServoPowered = false;
  bool wristServoPowered = false;
  bool gripperServoPowered = false;
  TwigHardwareConfig config;
  uint checksum = 0;
};

#endif