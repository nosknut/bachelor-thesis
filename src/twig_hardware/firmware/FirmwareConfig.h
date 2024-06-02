#ifndef FIRMWARE_CONFIG_h
#define FIRMWARE_CONFIG_h

const int8_t I2C_ADDRESS = 30;
const unsigned long SERIAL_BAUD_RATE = 115200;

const int SERVO_STATIONARY_SIGNAL = 1500;

const int INITIAL_MIN_ENCODER_MAGNITUDE = 10;

const int POSITION_LOWPASS_FILTER_ORDER = 2;
const float POSITION_LOWPASS_FILTER_CUTOFF = 2.0;

// To protect the system in the event of a connection loss,
// the system will stop moving if it does not detect activity
// within the specified number of milliseconds.
const unsigned long INITIAL_CONNECTION_TIMEOUT = 200;

const int8_t RASPBERRY_PI_SDA_PIN = 20;
const int8_t RASPBERRY_PI_SCL_PIN = 21;

// https://www.elfadistrelec.no/no/i2c-multiplexer-adafruit-2717/p/30091194
const uint32_t MULTIPLEXER_FREQUENCY = 100e3;
const uint32_t MULTIPLEXER_TIMEOUT = 500;
const int8_t MULTIPLEXER_ADDRESS = 0x70;
const int8_t MULTIPLEXER_SCL_PIN = 19;
const int8_t MULTIPLEXER_SDA_PIN = 18;

const int8_t SHOULDER_ENCODER_CHANNEL = 3;
const int8_t WRIST_ENCODER_CHANNEL = 2;
const int8_t GRIPPER_ENCODER_CHANNEL = 4;

const int8_t SHOULDER_CURRENT_PIN = A0;
const int8_t WRIST_CURRENT_PIN = A1;
const int8_t GRIPPER_CURRENT_PIN = A2;

const int8_t SESSION_ID_RANDOM_SEED_PIN = A2;

const int8_t SHOULDER_SERVO_PIN = 3;
const int8_t WRIST_SERVO_PIN = 12;
const int8_t GRIPPER_SERVO_PIN = 8;

const int8_t SHOULDER_SERVO_RELAY_PIN = 9;
const int8_t WRIST_SERVO_RELAY_PIN = 11;
const int8_t GRIPPER_SERVO_RELAY_PIN = 10;

#endif