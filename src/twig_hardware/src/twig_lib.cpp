#include "twig_hardware/twig_lib.hpp"

twig_hardware::TwigLib::TwigLib(int i2c_bus, int i2c_address, std::string i2c_device)
: i2c_bus(i2c_bus), i2c_address(i2c_address), i2c_device(i2c_device)
{
}

twig_hardware::TwigLib::~TwigLib()
{
  i2c_close_connection();
}

// I2c

bool twig_hardware::TwigLib::i2c_open_connection()
{
  // TODO: Remove the line below and replace with a timeout
  // Ensure there is no pre-existing connection
  i2c_close_connection();
  int connection = open(i2c_device.c_str(), O_RDWR);

  if (connection < 0) {
    return false;
  }

  if (ioctl(connection, I2C_SLAVE, i2c_address) < 0) {
    return false;
  }

  i2c_connection = connection;

  return true;
}

bool twig_hardware::TwigLib::i2c_close_connection()
{
  if (i2c_connection != -1) {
    close(i2c_connection);
    i2c_connection = -1;
    return true;
  }
  return false;
}

bool twig_hardware::TwigLib::i2c_send(std::byte * data, int data_size)
{
  i2c_open_connection();

  // Send the I2C message
  if (write(i2c_connection, data, data_size) != data_size) {
    i2c_close_connection();
    return false;
  }

  i2c_close_connection();

  return true;
}

bool twig_hardware::TwigLib::i2c_read(std::byte * buffer, int data_size)
{
  i2c_open_connection();

  // Request the state from the i2c device
  if (read(i2c_connection, buffer, data_size) != data_size) {
    i2c_close_connection();
    return false;
  }

  i2c_close_connection();

  return true;
}

// Sync

bool twig_hardware::TwigLib::write_command(int max_retries, bool force)
{
  if (!has_unpushed_commands_ && !force) {
    return true;
  }

  int fails = 0;
  while (true) {
    if (i2c_send(reinterpret_cast<std::byte *>(&command), sizeof(command))) {
      has_unpushed_commands_ = false;
      return true;
    }
    fails++;
    if (fails > max_retries) {
      return false;
    }
  }
}

bool twig_hardware::TwigLib::read_state(int max_retries)
{
  int fails = 0;
  while (true) {
    if (i2c_read(reinterpret_cast<std::byte *>(&state), sizeof(twig_hardware::TwigState))) {
      return true;
    }
    fails++;
    if (fails > max_retries) {
      return false;
    }
  }
}

// Utility

double twig_hardware::TwigLib::raw_to_radians(int16_t raw)
{
  return (((double) raw) / 4096.0) * 2 * M_PI;
}

double twig_hardware::TwigLib::degrees_to_radians(double degrees)
{
  return degrees * M_PI / 180.0;
}

// TODO: Implement voltage estimation
double twig_hardware::TwigLib::raw_to_voltage(int16_t raw)
{
  return ((double) raw) / 1024.0;
}

// TODO: Implement current estimation
double twig_hardware::TwigLib::raw_to_current(int16_t raw)
{
  return ((double) raw) / 1024.0;
}

// TODO: Implement effort estimation
double twig_hardware::TwigLib::current_to_effort(double current)
{
  return current;
}

// Activate

void twig_hardware::TwigLib::activate_shoulder_servo()
{
  stop_shoulder_servo();
  command.shoulderServoPowered = true;
  has_unpushed_commands_ = true;
}

void twig_hardware::TwigLib::activate_wrist_servo()
{
  stop_wrist_servo();
  command.wristServoPowered = true;
  has_unpushed_commands_ = true;
}

void twig_hardware::TwigLib::activate_gripper_servo()
{
  stop_gripper_servo();
  command.gripperServoPowered = true;
  has_unpushed_commands_ = true;
}

void twig_hardware::TwigLib::activate_all_servos()
{
  activate_shoulder_servo();
  activate_wrist_servo();
  activate_gripper_servo();
}

// Deactivate

void twig_hardware::TwigLib::deactivate_shoulder_servo()
{
  stop_shoulder_servo();
  command.shoulderServoPowered = false;
  has_unpushed_commands_ = true;
}

void twig_hardware::TwigLib::deactivate_wrist_servo()
{
  stop_wrist_servo();
  command.wristServoPowered = false;
  has_unpushed_commands_ = true;
}

void twig_hardware::TwigLib::deactivate_gripper_servo()
{
  stop_gripper_servo();
  command.gripperServoPowered = false;
  has_unpushed_commands_ = true;
}

void twig_hardware::TwigLib::deactivate_all_servos()
{
  deactivate_shoulder_servo();
  deactivate_wrist_servo();
  deactivate_gripper_servo();
}

// Set velocity

// TODO: Implement velocity control

void twig_hardware::TwigLib::set_shoulder_servo_velocity(double velocity)
{
  auto new_value = velocity * 600;
  if (command.shoulder != new_value) {
    has_unpushed_commands_ = true;
    command.shoulder = new_value;
  }
}

void twig_hardware::TwigLib::set_wrist_servo_velocity(double velocity)
{
  auto new_value = velocity * 600;
  if (command.wrist != new_value) {
    has_unpushed_commands_ = true;
    command.wrist = new_value;
  }
}

void twig_hardware::TwigLib::set_gripper_servo_velocity(double velocity)
{
  auto new_value = velocity * 600;
  if (command.gripper != new_value) {
    has_unpushed_commands_ = true;
    command.gripper = new_value;
  }
}

// Stop

void twig_hardware::TwigLib::stop_shoulder_servo()
{
  set_shoulder_servo_velocity(0);
}

void twig_hardware::TwigLib::stop_wrist_servo()
{
  set_wrist_servo_velocity(0);
}

void twig_hardware::TwigLib::stop_gripper_servo()
{
  set_gripper_servo_velocity(0);
}

void twig_hardware::TwigLib::stop_all_servos()
{
  stop_shoulder_servo();
  stop_wrist_servo();
  stop_gripper_servo();
}

// Get activation status

double twig_hardware::TwigLib::get_shoulder_servo_activation_status()
{
  return state.shoulderServoPowered;
}

double twig_hardware::TwigLib::get_wrist_servo_activation_status()
{
  return state.wristServoPowered;
}

double twig_hardware::TwigLib::get_gripper_servo_activation_status()
{
  return state.gripperServoPowered;
}

// Get current

double twig_hardware::TwigLib::get_shoulder_servo_current()
{
  return raw_to_current(state.shoulderCurrent);
}

double twig_hardware::TwigLib::get_wrist_servo_current()
{
  return raw_to_current(state.wristCurrent);
}

double twig_hardware::TwigLib::get_gripper_servo_current()
{
  return raw_to_current(state.gripperCurrent);
}

// Get voltage

double twig_hardware::TwigLib::get_shoulder_servo_voltage()
{
  return raw_to_voltage(state.shoulderVoltage);
}

double twig_hardware::TwigLib::get_wrist_servo_voltage()
{
  return raw_to_voltage(state.wristVoltage);
}

// Get velocity

double twig_hardware::TwigLib::get_shoulder_servo_velocity()
{
  return degrees_to_radians(state.shoulderVelocity);
}

double twig_hardware::TwigLib::get_wrist_servo_velocity()
{
  return degrees_to_radians(state.wristVelocity);
}

double twig_hardware::TwigLib::get_gripper_servo_velocity()
{
  return degrees_to_radians(state.gripperVelocity);
}

// Get position

double twig_hardware::TwigLib::get_shoulder_servo_position()
{
  return raw_to_radians(state.shoulderPosition);
}

double twig_hardware::TwigLib::get_wrist_servo_position()
{
  return raw_to_radians(state.wristPosition);
}

double twig_hardware::TwigLib::get_gripper_servo_position()
{
  return raw_to_radians(state.gripperPosition);
}

// Get encoder magnitude

double twig_hardware::TwigLib::get_shoulder_encoder_magnitude()
{
  return state.shoulderEncoderMagnitude;
}

double twig_hardware::TwigLib::get_wrist_encoder_magnitude()
{
  return state.wristEncoderMagnitude;
}

double twig_hardware::TwigLib::get_gripper_encoder_magnitude()
{
  return state.gripperEncoderMagnitude;
}

// Get effort

double twig_hardware::TwigLib::get_shoulder_servo_effort()
{
  return current_to_effort(get_shoulder_servo_current());
}

double twig_hardware::TwigLib::get_wrist_servo_effort()
{
  return current_to_effort(get_wrist_servo_current());
}

double twig_hardware::TwigLib::get_gripper_servo_effort()
{
  return current_to_effort(get_gripper_servo_current());
}
