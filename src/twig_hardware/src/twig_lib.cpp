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

bool twig_hardware::TwigLib::write_command()
{
  return i2c_send(reinterpret_cast<std::byte *>(&command), sizeof(command));
}

bool twig_hardware::TwigLib::read_state()
{
  return i2c_read(reinterpret_cast<std::byte *>(&state), sizeof(state));
}

bool twig_hardware::TwigLib::update_velocities(double delta_time)
{
  bool updated = false;

  if (delta_time < velocity_timeout) {
    shoulder_velocity = (get_shoulder_servo_position() - previous_shoulder_position) / delta_time;
    wrist_velocity = (get_wrist_servo_position() - previous_wrist_position) / delta_time;
    gripper_velocity = (get_gripper_servo_position() - previous_gripper_position) / delta_time;

    updated = true;
  }

  previous_shoulder_position = get_shoulder_servo_position();
  previous_wrist_position = get_wrist_servo_position();
  previous_gripper_position = get_gripper_servo_position();

  return updated;
}

// Utility

double raw_to_radians(int16_t raw)
{
  return (((double) raw) / 4096.0) * 2 * M_PI;
}

// TODO: Implement voltage estimation
double raw_to_voltage(int16_t raw)
{
  return ((double) raw) / 1024.0;
}

// TODO: Implement current estimation
double raw_to_current(int16_t raw)
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
}

void twig_hardware::TwigLib::activate_wrist_servo()
{
  stop_wrist_servo();
  command.wristServoPowered = true;
}

void twig_hardware::TwigLib::activate_gripper_servo()
{
  stop_gripper_servo();
  command.gripperServoPowered = true;
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
}

void twig_hardware::TwigLib::deactivate_wrist_servo()
{
  stop_wrist_servo();
  command.wristServoPowered = false;
}

void twig_hardware::TwigLib::deactivate_gripper_servo()
{
  stop_gripper_servo();
  command.gripperServoPowered = false;
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
  command.shoulder = velocity;
}

void twig_hardware::TwigLib::set_wrist_servo_velocity(double velocity)
{
  command.wrist = velocity;
}

void twig_hardware::TwigLib::set_gripper_servo_velocity(double velocity)
{
  command.gripper = velocity;
}

// Stop

void twig_hardware::TwigLib::stop_shoulder_servo()
{
  command.shoulder = 0;
}

void twig_hardware::TwigLib::stop_wrist_servo()
{
  command.wrist = 0;
}

void twig_hardware::TwigLib::stop_gripper_servo()
{
  command.gripper = 0;
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
  return shoulder_velocity;
}

double twig_hardware::TwigLib::get_wrist_servo_velocity()
{
  return wrist_velocity;
}

double twig_hardware::TwigLib::get_gripper_servo_velocity()
{
  return gripper_velocity;
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
