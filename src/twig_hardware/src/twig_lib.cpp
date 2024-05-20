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

void twig_hardware::TwigLib::set_hardware_config(twig_hardware::TwigHardwareConfig hardware_config)
{
  command.config = hardware_config;
  has_unpushed_commands_ = true;
}

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
    twig_hardware::TwigState payload;
    if (i2c_read(reinterpret_cast<std::byte *>(&payload), sizeof(twig_hardware::TwigState))) {
      if (payload.integrityCheck == 85) {
        state = payload;
        return true;
      }
    }
    fails++;
    if (fails > max_retries) {
      return false;
    }
  }
}

// Hardware Reboot

bool twig_hardware::TwigLib::driver_rebooted()
{
  return (command.sessionId == 0) || (state.sessionId == 0);
}

bool twig_hardware::TwigLib::hardware_rebooted()
{
  return !driver_rebooted() && (state.sessionId != command.sessionId);
}

void twig_hardware::TwigLib::acknowledge_hardware_reboot()
{
  command.sessionId = state.sessionId;
  has_unpushed_commands_ = true;
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

bool twig_hardware::TwigLib::respects_position_limits(
  double position, double velocity,
  Range limits)
{
  if (velocity > 0 && position >= limits.max) {
    return false;
  }

  if (velocity < 0 && position <= limits.min) {
    return false;
  }

  return true;
}

// Converts the range [0, 2 * M_PI] to [-M_PI, M_PI]
double twig_hardware::TwigLib::full_angle_to_center_angle(double angle)
{
  return angle - M_PI;
}

// Applies an angular offset to an angle in the range [-M_PI, M_PI]
double twig_hardware::TwigLib::apply_angular_offset(double angle, double offset)
{
  double new_angle = angle + offset;
  if (new_angle > M_PI) {
    return new_angle - 2 * M_PI;
  }

  if (new_angle < -M_PI) {
    return new_angle + 2 * M_PI;
  }

  return new_angle;
}

// Current conversion when using the PSM current sensor
double twig_hardware::TwigLib::raw_to_current_psm(uint16_t raw)
{
  double input_voltage = raw * (microcontroller_ref_voltage / 1024.0);
  double corrected_input_voltage = input_voltage - psm_offset;
  double current = corrected_input_voltage * psm_sensitivity;

  return current;
}

// Current conversion when using the ACS70331 current sensor
double twig_hardware::TwigLib::raw_to_current_acs(uint16_t raw)
{
  double input_voltage = raw * (microcontroller_ref_voltage / 1024.0);
  double corrected_input_voltage = input_voltage - acs_offset;

  // Compared to the PSM the ACS sensor has a different formula for current calculation
  double current = corrected_input_voltage / acs_sensitivity;

  return current;
}

// TODO: Implement current estimation
uint16_t twig_hardware::TwigLib::current_to_raw_psm(double current)
{
  double corrected_input_voltage = current / psm_sensitivity;
  double input_voltage = corrected_input_voltage + psm_offset;
  return input_voltage * (1024.0 / microcontroller_ref_voltage);
}

// TODO: Implement current estimation
uint16_t twig_hardware::TwigLib::current_to_raw_acs(double current)
{
  double corrected_input_voltage = current * acs_sensitivity;
  double input_voltage = corrected_input_voltage + acs_offset;
  return input_voltage * (1024.0 / microcontroller_ref_voltage);
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
  auto new_value = velocity * -200;
  if (!respects_position_limits(
      get_shoulder_servo_position(),
      new_value,
      jointConfig.shoulderLimits
  ))
  {
    new_value = 0;
  }
  if (command.shoulder != new_value) {
    has_unpushed_commands_ = true;
    command.shoulder = new_value;
  }
}

void twig_hardware::TwigLib::set_wrist_servo_velocity(double velocity)
{
  auto new_value = velocity * -200;
  if (command.wrist != new_value) {
    has_unpushed_commands_ = true;
    command.wrist = new_value;
  }
}

void twig_hardware::TwigLib::set_gripper_servo_velocity(double velocity)
{
  auto new_value = velocity * -200;
  if (!respects_position_limits(
      get_gripper_servo_position(),
      new_value,
      jointConfig.gripperLimits
  ))
  {
    new_value = 0;
  }
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
  return raw_to_current_psm(state.shoulderCurrent);
}

double twig_hardware::TwigLib::get_wrist_servo_current()
{
  return raw_to_current_psm(state.wristCurrent);
}

double twig_hardware::TwigLib::get_gripper_servo_current()
{
  return raw_to_current_acs(state.gripperCurrent);
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
  return apply_angular_offset(
    full_angle_to_center_angle(raw_to_radians(state.shoulderPosition)),
    jointConfig.shoulderOffset
  );
}

double twig_hardware::TwigLib::get_wrist_servo_position()
{
  return apply_angular_offset(
    full_angle_to_center_angle(raw_to_radians(state.wristPosition)),
    jointConfig.wristOffset
  );
}

double twig_hardware::TwigLib::get_gripper_servo_position()
{
  return apply_angular_offset(
    full_angle_to_center_angle(raw_to_radians(state.gripperPosition)),
    jointConfig.gripperOffset
  );
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
