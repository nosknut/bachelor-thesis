#include <Wire.h>
#include <Servo.h>
#include<avr/wdt.h> 
#include <SoftI2C.h>
#include "AS5600.h"
#include "RunningMedian.h"

const bool DEBUG_MAGNETS = false;

const int8_t I2C_ADDRESS = 30;
const unsigned long SERIAL_BAUD_RATE = 115200;

const int SERVO_STATIONARY_SIGNAL = 1500;

const int MIN_MAGNITUDE = 10;

// To protect the system in the event of a connection loss,
// the system will stop moving if it does not detect activity
// within the specified number of milliseconds.
const unsigned long CONNECTION_TIMEOUT = 200;

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

// Attribute packed is used to ensure the struct has the same
// shape across different architectures.
// This allows it to be sent through the i2c bus.
// https://stackoverflow.com/questions/21092415/force-c-structure-to-pack-tightly
// https://www.geeksforgeeks.org/how-to-avoid-structure-padding-in-c/
struct __attribute__((packed)) TwigState
{
  int16_t wristPosition = 0;
  int16_t gripperPosition = 0;
  int16_t shoulderPosition = 0;
  float wristVelocity = 0;
  float gripperVelocity = 0;
  float shoulderVelocity = 0;
  int16_t wristCurrent = 0;
  int16_t gripperCurrent = 0;
  int16_t shoulderCurrent = 0;
  int16_t wristVoltage = 0;
  int16_t shoulderVoltage = 0;
  bool shoulderServoPowered = false;
  bool wristServoPowered = false;
  bool gripperServoPowered = false;
};

struct AngularSpeedTracker
{
  RunningMedian angleFilter = RunningMedian(2);
  RunningMedian speedMedianFilter = RunningMedian(3);
  RunningMedian speedFilter = RunningMedian(3);

  unsigned long prevTime = micros();
  double prevAngle = 0;
  unsigned long period = 20e3;

  double updatePeriodically(double angle)
  {
    if ((micros() - prevTime) >= period) {
      return update(angle);
    }
    return speedFilter.getAverage();
  }

  // Based on code in AS5600 lib
  double update(double angle)
  {
    unsigned long now = micros();
    unsigned long deltaT = now - prevTime;

    angleFilter.add(angle);
    double deltaA = angleFilter.getMedian() - prevAngle;


    //  assumption is that there is no more than 180° rotation
    //  between two consecutive measurements.
    //  => at least two measurements per rotation (preferred 4).
    if (deltaA > 2048) {
      deltaA -= 4096;
    }

    if (deltaA < -2048) {
      deltaA += 4096;
    }

    speedMedianFilter.add(((deltaA * 1e6) / deltaT) * AS5600_RAW_TO_DEGREES);
    speedFilter.add(speedMedianFilter.getMedian());

    prevAngle = angleFilter.getMedian();
    prevTime = now;

    return speedFilter.getAverage();
  }
};

SoftI2C shoulderEncoderI2cBus = SoftI2C(SHOULDER_ENCODER_SDA_PIN, SHOULDER_ENCODER_SCL_PIN);
SoftI2C wristEncoderI2cBus = SoftI2C(WRIST_ENCODER_SDA_PIN, WRIST_ENCODER_SCL_PIN);
SoftI2C gripperEncoderI2cBus = SoftI2C(GRIPPER_ENCODER_SDA_PIN, GRIPPER_ENCODER_SCL_PIN);

AS5600 shoulderEncoder(&shoulderEncoderI2cBus);
AS5600 wristEncoder(&wristEncoderI2cBus);
AS5600 gripperEncoder(&gripperEncoderI2cBus);

AngularSpeedTracker shoulderSpeedTracker;
AngularSpeedTracker wristSpeedTracker;
AngularSpeedTracker gripperSpeedTracker;

Servo shoulderServo;
Servo wristServo;
Servo gripperServo;

TwigCommand twigCommand;
TwigState twigState;

unsigned long connectionTimer = 0;
unsigned long logTimer = 0;

unsigned long sent = 0;
unsigned long received = 0;

void resetCommand()
{
  twigCommand.shoulder = 0;
  twigCommand.wrist = 0;
  twigCommand.gripper = 0;
  twigCommand.shoulderServoPowered = false;
  twigCommand.wristServoPowered = false;
  twigCommand.gripperServoPowered = false;
}

void writeCommand()
{
  shoulderServo.writeMicroseconds(SERVO_STATIONARY_SIGNAL + twigCommand.shoulder);
  wristServo.writeMicroseconds(SERVO_STATIONARY_SIGNAL + twigCommand.wrist);
  gripperServo.writeMicroseconds(SERVO_STATIONARY_SIGNAL + twigCommand.gripper);

  digitalWrite(SHOULDER_SERVO_RELAY_PIN, twigCommand.shoulderServoPowered ? HIGH : LOW);
  digitalWrite(WRIST_SERVO_RELAY_PIN, twigCommand.wristServoPowered ? HIGH : LOW);
  digitalWrite(GRIPPER_SERVO_RELAY_PIN, twigCommand.gripperServoPowered ? HIGH : LOW);
}

void onCommand(int length)
{
  Wire.readBytes((uint8_t *)&twigCommand, length);
  connectionTimer = millis();
  received++;
}

void updateVelocity()
{
  twigState.wristVelocity = wristSpeedTracker.updatePeriodically(twigState.wristPosition);
  twigState.gripperVelocity = gripperSpeedTracker.updatePeriodically(twigState.gripperPosition);
  twigState.shoulderVelocity = shoulderSpeedTracker.updatePeriodically(twigState.shoulderPosition);
}

void readAngle(AS5600 encoder, int16_t & currentAngle, String name)
{
  if (encoder.readMagnitude() > MIN_MAGNITUDE) {
    currentAngle = encoder.readAngle();
  } else if (DEBUG_MAGNETS) {
    Serial.println("ERROR: " + name + " encoder magnet is too weak");
  }
}

void readState()
{
  twigState.wristCurrent = analogRead(WRIST_CURRENT_PIN);
  twigState.gripperCurrent = analogRead(GRIPPER_CURRENT_PIN);
  twigState.shoulderCurrent = analogRead(SHOULDER_CURRENT_PIN);

  twigState.wristVoltage = analogRead(WRIST_VOLTAGE_PIN);
  twigState.shoulderVoltage = analogRead(SHOULDER_VOLTAGE_PIN);

  readAngle(wristEncoder, twigState.wristPosition, "wrist");
  readAngle(gripperEncoder, twigState.gripperPosition, "gripper");
  readAngle(shoulderEncoder, twigState.shoulderPosition, "shoulder");

  twigState.wristServoPowered = digitalRead(WRIST_SERVO_RELAY_PIN) == HIGH;
  twigState.shoulderServoPowered = digitalRead(SHOULDER_SERVO_RELAY_PIN) == HIGH;
  twigState.gripperServoPowered = digitalRead(GRIPPER_SERVO_RELAY_PIN) == HIGH;
}

void onStateRequest()
{
  Wire.write((uint8_t *)&twigState, sizeof(TwigState));
  connectionTimer = millis();
  sent++;
}

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  Serial.println("Starting ...");

  // https://www.electronicwings.com/arduino/watchdog-in-arduino
  wdt_enable(WDTO_1S);
  
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

String leadingSpaces(String val, int length)
{
  int valLen = val.length();
  String out = "";
  for (int i = 0; i < length - valLen; i++) {
    out += " ";
  }
  out += val;
  return out;
}

void printLog()
{
  Serial.print("Sending at: ");
  Serial.print(sent * 10);
  Serial.print("Hz, Receiving at: ");
  Serial.print(received * 10);
  Serial.print("Hz, ");
  Serial.print(leadingSpaces(String(twigCommand.shoulder), 7));
  Serial.print(", ");
  Serial.print(leadingSpaces(String(twigCommand.wrist), 7));
  Serial.print(", ");
  Serial.print(leadingSpaces(String(twigCommand.gripper), 7));
  Serial.print(", ");
  Serial.print(twigCommand.shoulderServoPowered);
  Serial.print(", ");
  Serial.print(twigCommand.wristServoPowered);
  Serial.print(", ");
  Serial.println(twigCommand.gripperServoPowered);
}

void updateLog()
{
  if ((millis() - logTimer) >= 100) {
    printLog();
    sent = 0;
    received = 0;
    logTimer = millis();
  }
}

void updateConnectionTimer()
{
  if ((millis() - connectionTimer) > CONNECTION_TIMEOUT) {
    resetCommand();
  }
}

void loop()
{
  readState();
  updateVelocity();
  updateLog();
  updateConnectionTimer();
  writeCommand();
  delay(1);
  wdt_reset();
}
