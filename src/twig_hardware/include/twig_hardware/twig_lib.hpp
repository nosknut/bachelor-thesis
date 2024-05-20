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
  uint16_t maxShoulderCurrent = 0;
  uint16_t maxWristCurrent = 0;
  uint16_t maxGripperCurrent = 0;
  int16_t maxCurrentDuration = 0;
  int16_t maxCurrentCooldownDuration = 0;
  int16_t encoderMinMagnitude = 0;
};

// Attribute packed is used to ensure the struct has the same
// shape across different architectures.
// This allows it to be sent through the i2c bus.
// https://stackoverflow.com/questions/21092415/force-c-structure-to-pack-tightly
// https://www.geeksforgeeks.org/how-to-avoid-structure-padding-in-c/
struct __attribute__((packed)) TwigCommand
{
  int16_t integrityCheck = 85;
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
  int16_t integrityCheck = 85;
  int16_t sessionId = 0;
  int16_t wristPosition = 0;
  int16_t gripperPosition = 0;
  int16_t shoulderPosition = 0;
  float wristVelocity = 0;
  float gripperVelocity = 0;
  float shoulderVelocity = 0;
  uint16_t wristEncoderMagnitude = 0;
  uint16_t gripperEncoderMagnitude = 0;
  uint16_t shoulderEncoderMagnitude = 0;
  uint16_t wristCurrent = 0;
  uint16_t gripperCurrent = 0;
  uint16_t shoulderCurrent = 0;
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

  
  // Values from the microcontroller datasheet
  double microcontroller_ref_voltage = 5.0;

  // Values from the PSM datasheet
  double psm_sensitivity = 37.8788;
  double psm_offset = 0.33;

  // Values from the ACS datasheet
  double acs_sensitivity = 0.4;
  double acs_offset = 1.5;

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

  double raw_to_current_psm(uint16_t raw);
  double raw_to_current_acs(uint16_t raw);
  double current_to_effort(double current);

public:
  uint16_t current_to_raw_psm(double current);
  uint16_t current_to_raw_acs(double current);

// Sync
  void set_hardware_config(TwigHardwareConfig hardware_config);
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
