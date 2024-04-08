#ifndef TWIG_COMMAND_h
#define TWIG_COMMAND_h

struct TwigCommand
{
  int16_t wrist = 0;
  int16_t gripper = 0;
  int16_t shoulder = 0;
  bool shoulderServoPowered = false;
  bool wristServoPowered = false;
  bool gripperServoPowered = false;
};

#endif