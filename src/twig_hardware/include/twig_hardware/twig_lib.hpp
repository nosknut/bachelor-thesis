#ifndef TWIG_LIB_HPP
#define TWIG_LIB_HPP

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <linux/i2c-dev.h>
#include <map>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace twig_hardware
{

struct Range
{
  double min = 0;
  double max = 0;

  Range(double min, double max)
  : min(min), max(max) {}
};

struct TwigJointConfig
{
  Range gripperLimits = Range(-M_PI, M_PI);
  Range shoulderLimits = Range(-M_PI, M_PI);
  double shoulderOffset = 0;
  double gripperOffset = 0;
  double wristOffset = 0;
};

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
};

// Attribute packed is used to ensure the struct has the same
// shape across different architectures.
// This allows it to be sent through the i2c bus.
// https://stackoverflow.com/questions/21092415/force-c-structure-to-pack-tightly
// https://www.geeksforgeeks.org/how-to-avoid-structure-padding-in-c/
struct __attribute__((packed)) TwigState
{
  int16_t sessionId = 0;
  int16_t wristPosition = 0;
  int16_t gripperPosition = 0;
  int16_t shoulderPosition = 0;
  float wristVelocity = 0;
  float gripperVelocity = 0;
  float shoulderVelocity = 0;
  int16_t wristEncoderMagnitude = 0;
  int16_t gripperEncoderMagnitude = 0;
  int16_t shoulderEncoderMagnitude = 0;
  int16_t wristCurrent = 0;
  int16_t gripperCurrent = 0;
  int16_t shoulderCurrent = 0;
  int16_t wristVoltage = 0;
  int16_t shoulderVoltage = 0;
  bool shoulderServoPowered = false;
  bool wristServoPowered = false;
  bool gripperServoPowered = false;
};

class TwigLib
{
protected:
  int i2c_bus;
  int i2c_address;
  int i2c_connection = -1;
  std::string i2c_device;

  double shoulder_position_offset = 0;
  double wrist_position_offset = 0;
  double gripper_position_offset = 0;

  bool has_unpushed_commands_ = false;

public:
  TwigCommand command;
  TwigState state;
  TwigJointConfig jointConfig;

  TwigLib(int i2c_bus = 1, int i2c_address = 30, std::string i2c_device = "/dev/i2c-1");
  ~TwigLib();

protected:
// I2c
  bool i2c_open_connection();
  bool i2c_close_connection();
  bool i2c_send(std::byte * data, int data_size);
  bool i2c_read(std::byte * buffer, int data_size);

// Utility

  double degrees_to_radians(double degrees);
  double raw_to_radians(int16_t raw);

  bool respects_position_limits(double position, double velocity, Range limits);
  double full_angle_to_center_angle(double angle);
  double apply_angular_offset(double angle, double offset);

  double raw_to_voltage(int16_t raw);

  double raw_to_current(int16_t raw);
  double current_to_effort(double current);

public:
  int16_t current_to_raw(double current);

// Sync
  bool write_command(int max_retries = 1, bool force = false);
  bool read_state(int max_retries = 1);

// Hardware Reboot
  bool driver_rebooted();
  bool hardware_rebooted();
  void acknowledge_hardware_reboot();

// Activate

  void activate_shoulder_servo();
  void activate_wrist_servo();
  void activate_gripper_servo();
  void activate_all_servos();

// Deactivate

  void deactivate_shoulder_servo();
  void deactivate_wrist_servo();
  void deactivate_gripper_servo();
  void deactivate_all_servos();

// Set velocity

  void set_shoulder_servo_velocity(double velocity);
  void set_wrist_servo_velocity(double velocity);
  void set_gripper_servo_velocity(double velocity);

// Stop

  void stop_shoulder_servo();
  void stop_wrist_servo();
  void stop_gripper_servo();
  void stop_all_servos();

// Get activation status

  double get_shoulder_servo_activation_status();
  double get_wrist_servo_activation_status();
  double get_gripper_servo_activation_status();

// Get current

  double get_shoulder_servo_current();
  double get_wrist_servo_current();
  double get_gripper_servo_current();

// Get voltage

  double get_shoulder_servo_voltage();
  double get_wrist_servo_voltage();

// Get velocity

  double get_shoulder_servo_velocity();
  double get_wrist_servo_velocity();
  double get_gripper_servo_velocity();

// Get position

  double get_shoulder_servo_position();
  double get_wrist_servo_position();
  double get_gripper_servo_position();

// Get encoder magnitude

  double get_shoulder_encoder_magnitude();
  double get_wrist_encoder_magnitude();
  double get_gripper_encoder_magnitude();

// Get effort

  double get_shoulder_servo_effort();
  double get_wrist_servo_effort();
  double get_gripper_servo_effort();
};
}

#endif
