// How to flash the firmware to Raspberry Pi Pico from Arduino IDE:
// https://randomnerdtutorials.com/programming-raspberry-pi-pico-w-arduino-ide/#Raspberry-Pi-Pico-Boards-Manager
// Additional URL:
// https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
// Board manager:
// Raspberry Pi Pico/RP2040 by Earle F. Philhower, III Version 3.8.0 INSTALLED
// Serial port not showing up:
// https://forum.arduino.cc/t/raspberry-pi-pico-does-not-show-its-port-in-ide/850432/10

#include "FirmwareConfig.h"

#include <Wire.h>
#include <Servo.h>
#include "hardware/watchdog.h"
#include "TwigState.h"
#include "TwigCommand.h"
#include "TwigHardwareConfig.h"
#include "FusedServo.h"

// #define DEBUG_ENCODERS
#include "Encoder.h"

Encoder shoulderEncoder = Encoder(SHOULDER_ENCODER_SDA_PIN, SHOULDER_ENCODER_SCL_PIN, INITIAL_MIN_ENCODER_MAGNITUDE, "Shoulder");
Encoder wristEncoder = Encoder(WRIST_ENCODER_SDA_PIN, WRIST_ENCODER_SCL_PIN, INITIAL_MIN_ENCODER_MAGNITUDE, "Wrist");
Encoder gripperEncoder = Encoder(GRIPPER_ENCODER_SDA_PIN, GRIPPER_ENCODER_SCL_PIN, INITIAL_MIN_ENCODER_MAGNITUDE, "Gripper");

FusedServo shoulderServo(SHOULDER_SERVO_PIN, SHOULDER_SERVO_RELAY_PIN);
FusedServo wristServo(WRIST_SERVO_PIN, WRIST_SERVO_RELAY_PIN);
FusedServo gripperServo(GRIPPER_SERVO_PIN, GRIPPER_SERVO_RELAY_PIN);

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

  shoulderEncoder.minMagnitude = config.encoderMinMagnitude;
  wristEncoder.minMagnitude = config.encoderMinMagnitude;
  gripperEncoder.minMagnitude = config.encoderMinMagnitude;

  shoulderServo.fuse.maxCurrent = config.maxCurrent;
  wristServo.fuse.maxCurrent = config.maxCurrent;
  gripperServo.fuse.maxCurrent = config.maxCurrent;

  shoulderServo.fuse.maxCurrentDuration = config.maxCurrentDuration;
  wristServo.fuse.maxCurrentDuration = config.maxCurrentDuration;
  gripperServo.fuse.maxCurrentDuration = config.maxCurrentDuration;

  shoulderServo.fuse.maxCurrentCooldownDuration = config.maxCurrentCooldownDuration;
  wristServo.fuse.maxCurrentCooldownDuration = config.maxCurrentCooldownDuration;
  gripperServo.fuse.maxCurrentCooldownDuration = config.maxCurrentCooldownDuration;
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
  TwigCommand payload;
  Wire.readBytes((uint8_t *)&payload, length);
  if (payload.integrityCheck != 85) {
    Serial.println("Command integrity check failed");
    return;
  }
  twigCommand = payload;
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

  wristEncoder.update();
  shoulderEncoder.update();
  gripperEncoder.update();

  twigState.wristPosition = wristEncoder.values.angle;
  twigState.wristVelocity = wristEncoder.values.velocity;
  twigState.wristEncoderMagnitude = wristEncoder.values.magnitude;

  twigState.shoulderPosition = shoulderEncoder.values.angle;
  twigState.shoulderVelocity = shoulderEncoder.values.velocity;
  twigState.shoulderEncoderMagnitude = shoulderEncoder.values.magnitude;

  twigState.gripperPosition = gripperEncoder.values.angle;
  twigState.gripperVelocity = gripperEncoder.values.velocity;
  twigState.gripperEncoderMagnitude = gripperEncoder.values.magnitude;

  twigState.wristServoPowered = wristServo.relayState;
  twigState.shoulderServoPowered = shoulderServo.relayState;
  twigState.gripperServoPowered = gripperServo.relayState;
}

void onStateRequest()
{
  Wire.write((uint8_t *)&twigState, sizeof(TwigState));
  if (verify_session_id()) connectionTimer = millis();
  sent++;
}

void setupOutputs()
{
  shoulderServo.begin();
  wristServo.begin();
  gripperServo.begin();
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
  Serial.print(leadingSpaces(String(twigState.shoulderEncoderMagnitude), 7));
  Serial.print(", ");
  Serial.print(leadingSpaces(String(twigState.wristEncoderMagnitude), 7));
  Serial.print(", ");
  Serial.print(leadingSpaces(String(twigState.gripperEncoderMagnitude), 7));
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
    Serial.println("Connection timeout");
  }
}

void setup()
{
  // Used to detect that the microcontroller rebooted
  randomSeed(analogRead(SESSION_ID_RANDOM_SEED_PIN));
  twigState.sessionId = random(1, 10000);

  setupOutputs();
  resetCommand();

  Serial.begin(SERIAL_BAUD_RATE);

  Serial.println("Starting ...");

  // https://github.com/raspberrypi/pico-examples/blob/master/watchdog/hello_watchdog/hello_watchdog.c
  // Enable the watchdog, requiring the watchdog to be updated every 100ms or the chip will reboot
  // second arg is pause on debug which means the watchdog will pause when stepping through code
  watchdog_enable(1000, 1);

  Wire.setSCL(RASPBERRY_PI_SCL_PIN);
  Wire.setSDA(RASPBERRY_PI_SDA_PIN);
  
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onCommand);
  Wire.onRequest(onStateRequest);

  shoulderEncoder.begin();
  wristEncoder.begin();
  gripperEncoder.begin();

  // Read the states and give the remote system some time to connect
  readState();
  delay(100);
}

void loop()
{
    readState();
    updateLog();
    updateConnectionTimer();
    writeCommand();
    watchdog_update();
}