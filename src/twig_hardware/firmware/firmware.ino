#include <Wire.h>
#include <Servo.h>
#include <SoftI2C.h>
#include "AS5600.h"

const int8_t I2C_ADDRESS = 30;
const int8_t SERIAL_BAUD_RATE = 115200;

const int8_t SHOULDER_ENCODER_SDA_PIN = 4;
const int8_t SHOULDER_ENCODER_SCL_PIN = 2;

const int8_t WRIST_ENCODER_SDA_PIN = 7;
const int8_t WRIST_ENCODER_SCL_PIN = 2;

const int8_t GRIPPER_ENCODER_SDA_PIN = 8;
const int8_t GRIPPER_ENCODER_SCL_PIN = 2;

const int8_t SHOULDER_CURRENT_PIN = A0;
const int8_t WRIST_CURRENT_PIN = A1;
const int8_t GRIPPER_CURRENT_PIN = A2;

const int8_t SHOULDER_VOLTAGE_PIN = A3;
const int8_t WRIST_VOLTAGE_PIN = A6;

const int8_t SHOULDER_SERVO_PIN = 3;
const int8_t WRIST_SERVO_PIN = 5;
const int8_t GRIPPER_SERVO_PIN = 6;

const int8_t SHOULDER_SERVO_RELAY_PIN = 9;
const int8_t WRIST_SERVO_RELAY_PIN = 10;
const int8_t GRIPPER_SERVO_RELAY_PIN = 11;

struct TwigCommand
{
    int16_t wrist = 0;
    int16_t gripper = 0;
    int16_t shoulder = 0;
    bool shoulderServoPowered = false;
    bool wristServoPowered = false;
    bool gripperServoPowered = false;
};

struct TwigState
{
    int16_t wristPosition = 0;
    int16_t gripperPosition = 0;
    int16_t shoulderPosition = 0;
    int8_t wristCurrent = 0;
    int8_t gripperCurrent = 0;
    int8_t shoulderCurrent = 0;
    int8_t wristVoltage = 0;
    int8_t shoulderVoltage = 0;
    bool shoulderServoPowered = false;
    bool wristServoPowered = false;
    bool gripperServoPowered = false;
};

SoftI2C shoulderEncoderI2cBus = SoftI2C(SHOULDER_ENCODER_SDA_PIN, SHOULDER_ENCODER_SCL_PIN);
SoftI2C wristEncoderI2cBus = SoftI2C(WRIST_ENCODER_SDA_PIN, WRIST_ENCODER_SCL_PIN);
SoftI2C gripperEncoderI2cBus = SoftI2C(GRIPPER_ENCODER_SDA_PIN, GRIPPER_ENCODER_SCL_PIN);

AS5600 shoulderEncoder(&shoulderEncoderI2cBus);
AS5600 wristEncoder(&wristEncoderI2cBus);
AS5600 gripperEncoder(&gripperEncoderI2cBus);

Servo shoulderServo;
Servo wristServo;
Servo gripperServo;

TwigCommand twigCommand;
TwigState twigState;

void onCommand(int length)
{
    Wire.readBytes((uint8_t *)&twigCommand, length);

    shoulderServo.writeMicroseconds(twigCommand.shoulder);
    wristServo.writeMicroseconds(twigCommand.wrist);
    gripperServo.writeMicroseconds(twigCommand.gripper);

    digitalWrite(SHOULDER_SERVO_RELAY_PIN, twigCommand.shoulderServoPowered ? HIGH : LOW);
    digitalWrite(WRIST_SERVO_RELAY_PIN, twigCommand.wristServoPowered ? HIGH : LOW);
    digitalWrite(GRIPPER_SERVO_RELAY_PIN, twigCommand.gripperServoPowered ? HIGH : LOW);
}

void onStateRequest()
{
    twigState.wristCurrent = analogRead(WRIST_CURRENT_PIN);
    twigState.gripperCurrent = analogRead(GRIPPER_CURRENT_PIN);
    twigState.shoulderCurrent = analogRead(SHOULDER_CURRENT_PIN);

    twigState.wristVoltage = analogRead(WRIST_VOLTAGE_PIN);
    twigState.shoulderVoltage = analogRead(SHOULDER_VOLTAGE_PIN);

    twigState.wristPosition = wristEncoder.readAngle();
    twigState.gripperPosition = gripperEncoder.readAngle();
    twigState.shoulderPosition = shoulderEncoder.readAngle();

    twigState.wristServoPowered = digitalRead(WRIST_SERVO_RELAY_PIN) == HIGH;
    twigState.shoulderServoPowered = digitalRead(SHOULDER_SERVO_RELAY_PIN) == HIGH;
    twigState.gripperServoPowered = digitalRead(GRIPPER_SERVO_RELAY_PIN) == HIGH;

    Wire.write((uint8_t *)&twigState, sizeof(TwigState));
}

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(onCommand);
    Wire.onRequest(onStateRequest);

    shoulderEncoderI2cBus.begin();
    wristEncoderI2cBus.begin();
    gripperEncoderI2cBus.begin();

    pinMode(SHOULDER_SERVO_RELAY_PIN, OUTPUT);
    pinMode(WRIST_SERVO_RELAY_PIN, OUTPUT);
    pinMode(GRIPPER_SERVO_RELAY_PIN, OUTPUT);

    digitalWrite(SHOULDER_SERVO_RELAY_PIN, LOW);
    digitalWrite(WRIST_SERVO_RELAY_PIN, LOW);
    digitalWrite(GRIPPER_SERVO_RELAY_PIN, LOW);

    shoulderServo.attach(SHOULDER_SERVO_PIN);
    wristServo.attach(WRIST_SERVO_PIN);
    gripperServo.attach(GRIPPER_SERVO_PIN);
}

void loop()
{
    delay(100);
}
