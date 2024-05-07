#ifndef FIRMWARE_CONFIG_h
#define FIRMWARE_CONFIG_h

const int8_t I2C_ADDRESS = 30;
const unsigned long SERIAL_BAUD_RATE = 115200;

const int SERVO_STATIONARY_SIGNAL = 1500;

const int INITIAL_MIN_ENCODER_MAGNITUDE = 10;

const int POSITION_LOWPASS_FILTER_ORDER = 2;
const int POSITION_LOWPASS_FILTER_CUTOFF = 2;

// To protect the system in the event of a connection loss,
// the system will stop moving if it does not detect activity
// within the specified number of milliseconds.
const unsigned long INITIAL_CONNECTION_TIMEOUT = 200;

const int8_t SHOULDER_ENCODER_SDA_PIN = 7;
const int8_t SHOULDER_ENCODER_SCL_PIN = 2;

const int8_t WRIST_ENCODER_SDA_PIN = 4;
const int8_t WRIST_ENCODER_SCL_PIN = 2;

const int8_t GRIPPER_ENCODER_SDA_PIN = 8;
const int8_t GRIPPER_ENCODER_SCL_PIN = 2;

const int8_t SHOULDER_CURRENT_PIN = A0;
const int8_t WRIST_CURRENT_PIN = A1;
const int8_t GRIPPER_CURRENT_PIN = A2;

const int8_t SHOULDER_VOLTAGE_PIN = A3;
// TODO: Remove sensors from firmware that have no analog input
const int8_t WRIST_VOLTAGE_PIN = A3;

const int8_t SESSION_ID_RANDOM_SEED_PIN = A3;

const int8_t SHOULDER_SERVO_PIN = 3;
const int8_t WRIST_SERVO_PIN = 5;
const int8_t GRIPPER_SERVO_PIN = 6;

const int8_t SHOULDER_SERVO_RELAY_PIN = 9;
const int8_t WRIST_SERVO_RELAY_PIN = 10;
const int8_t GRIPPER_SERVO_RELAY_PIN = 11;

#endif