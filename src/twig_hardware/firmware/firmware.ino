#include "FirmwareConfig.h"

#include <Wire.h>
#include <Servo.h>
#include <avr/wdt.h>
#include "TwigState.h"
#include "TwigCommand.h"
#include "TwigHardwareConfig.h"
#include "FusedServo.h"

// #define DEBUG_ENCODERS
#include "Encoder.h"

Encoder shoulderEncoder(SHOULDER_ENCODER_SDA_PIN, SHOULDER_ENCODER_SCL_PIN, INITIAL_MIN_ENCODER_MAGNITUDE, "Shoulder");
Encoder wristEncoder(WRIST_ENCODER_SDA_PIN, WRIST_ENCODER_SCL_PIN, INITIAL_MIN_ENCODER_MAGNITUDE, "Wrist");
Encoder gripperEncoder(GRIPPER_ENCODER_SDA_PIN, GRIPPER_ENCODER_SCL_PIN, INITIAL_MIN_ENCODER_MAGNITUDE, "Gripper");

FusedServo shoulderServo(SHOULDER_SERVO_PIN, SHOULDER_SERVO_RELAY_PIN);
FusedServo wristServo(SHOULDER_SERVO_PIN, SHOULDER_SERVO_RELAY_PIN);
FusedServo gripperServo(SHOULDER_SERVO_PIN, SHOULDER_SERVO_RELAY_PIN);

TwigCommand twigCommand;
TwigState twigState;

unsigned long connectionTimeout = INITIAL_CONNECTION_TIMEOUT;
unsigned long connectionTimer = 0;
unsigned long logTimer = 0;

unsigned long sent = 0;
unsigned long received = 0;

void resetCommand()
{
  twigCommand.sessionId = twigState.sessionId;
  twigCommand.shoulder = 0;
  twigCommand.wrist = 0;
  twigCommand.gripper = 0;
  twigCommand.shoulderServoPowered = false;
  twigCommand.wristServoPowered = false;
  twigCommand.gripperServoPowered = false;
}

bool verify_session_id()
{
  return twigCommand.sessionId == twigState.sessionId;
}

void updateHardwareConfig() {
  TwigHardwareConfig &config = twigCommand.config;

  connectionTimeout = config.connectionTimeout;

  shoulderEncoder.minMagnitude = config.shoulderEncoderMinMagnitude;
  wristEncoder.minMagnitude = config.wristEncoderMinMagnitude;
  gripperEncoder.minMagnitude = config.gripperEncoderMinMagnitude;

  shoulderServo.fuse.maxCurrent = config.shoulderMaxCurrent;
  wristServo.fuse.maxCurrent = config.wristMaxCurrent;
  gripperServo.fuse.maxCurrent = config.gripperMaxCurrent;

  shoulderServo.fuse.maxCurrentDuration = config.shoulderMaxCurrentDuration;
  wristServo.fuse.maxCurrentDuration = config.wristMaxCurrentDuration;
  gripperServo.fuse.maxCurrentDuration = config.gripperMaxCurrentDuration;

  shoulderServo.fuse.maxCurrentCooldownDuration = config.shoulderMaxCurrentCooldownDuration;
  wristServo.fuse.maxCurrentCooldownDuration = config.wristMaxCurrentCooldownDuration;
  gripperServo.fuse.maxCurrentCooldownDuration = config.gripperMaxCurrentCooldownDuration;
}

void writeCommand()
{
  if (!verify_session_id()) return;
  shoulderServo.update(twigState.shoulderCurrent, twigCommand.shoulderServoPowered, twigCommand.shoulder);
  wristServo.update(twigState.wristCurrent, twigCommand.wristServoPowered, twigCommand.wrist);
  gripperServo.update(twigState.gripperCurrent, twigCommand.gripperServoPowered, twigCommand.gripper);
}

void onCommand(int length)
{
  Wire.readBytes((uint8_t *)&twigCommand, length);
  updateHardwareConfig();
  if (verify_session_id()) connectionTimer = millis();
  received++;
}

void readState()
{
  twigState.wristCurrent = analogRead(WRIST_CURRENT_PIN);
  twigState.gripperCurrent = analogRead(GRIPPER_CURRENT_PIN);
  twigState.shoulderCurrent = analogRead(SHOULDER_CURRENT_PIN);

  twigState.wristVoltage = analogRead(WRIST_VOLTAGE_PIN);
  twigState.shoulderVoltage = analogRead(SHOULDER_VOLTAGE_PIN);

  if (wristEncoder.update())
  {
    twigState.wristPosition = wristEncoder.values.angle;
    twigState.wristVelocity = wristEncoder.values.velocity;
  }
  twigState.wristEncoderMagnitude = wristEncoder.values.magnitude;

  if (shoulderEncoder.update())
  {
    twigState.shoulderPosition = shoulderEncoder.values.angle;
    twigState.shoulderVelocity = shoulderEncoder.values.velocity;
  }
  twigState.shoulderEncoderMagnitude = shoulderEncoder.values.magnitude;

  if (gripperEncoder.update())
  {
    twigState.gripperPosition = gripperEncoder.values.angle;
    twigState.gripperVelocity = gripperEncoder.values.velocity;
  }
  twigState.gripperEncoderMagnitude = gripperEncoder.values.magnitude;

  twigState.wristServoPowered = wristServo.relayState;
  twigState.shoulderServoPowered = shoulderServo.relayState;
  twigState.gripperServoPowered = gripperServo.relayState;
}

void onStateRequest()
{
  Wire.write((uint8_t *)&twigState, sizeof(TwigState));
  connectionTimer = millis();
  sent++;
}

void setupOutputs()
{
  shoulderServo.begin();
  wristServo.begin();
  gripperServo.begin();
}

void setup()
{
  // Used to detect that the microcontroller rebooted
  twigState.sessionId = random(1, 10000);

  setupOutputs();
  resetCommand();

  Serial.begin(SERIAL_BAUD_RATE);

  Serial.println("Starting ...");

  // https://www.electronicwings.com/arduino/watchdog-in-arduino
  wdt_enable(WDTO_1S);

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onCommand);
  Wire.onRequest(onStateRequest);

  shoulderEncoder.begin();
  wristEncoder.begin();
  gripperEncoder.begin();

  // Wait 100ms after startup to allow driver sync
  readState();
  delay(100);
}

String leadingSpaces(String val, int length)
{
  int valLen = val.length();
  String out = "";
  for (int i = 0; i < length - valLen; i++)
  {
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
  if ((millis() - logTimer) >= 100)
  {
    printLog();
    sent = 0;
    received = 0;
    logTimer = millis();
  }
}

void updateConnectionTimer()
{
  if ((millis() - connectionTimer) > connectionTimeout)
  {
    resetCommand();
  }
}

void loop()
{
  readState();
  updateLog();
  updateConnectionTimer();
  writeCommand();
  wdt_reset();
}
