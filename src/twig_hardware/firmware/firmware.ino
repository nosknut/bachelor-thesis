#include "FirmwareConfig.h"

#include <Wire.h>
#include <Servo.h>
#include <avr/wdt.h>
#include "TwigState.h"
#include "TwigCommand.h"
#include "TwigHardwareConfig.h"
#include "FusedServo.h"
#include "EncoderThread.h"
#include "Thread.h"

// #define DEBUG_ENCODERS
#include "Encoder.h"

EncoderThread encoderThread;

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

  encoderThread.shoulderEncoder.minMagnitude = config.encoderMinMagnitude;
  encoderThread.wristEncoder.minMagnitude = config.encoderMinMagnitude;
  encoderThread.gripperEncoder.minMagnitude = config.encoderMinMagnitude;

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
  Wire.readBytes((uint8_t *)&twigCommand, length);
  updateHardwareConfig();
  if (verify_session_id()) connectionTimer = millis();
  received++;
}

uint16_t downsize10Bit(uint16_t input)
{
  return map(input, 0, 1023, 0, 255);
}

uint16_t downsize12Bit(uint16_t input)
{
  return map(input, 0, 4095, 0, 255);
}

void readState()
{
  twigState.wristCurrent = downsize10Bit(analogRead(WRIST_CURRENT_PIN));
  twigState.gripperCurrent = downsize10Bit(analogRead(GRIPPER_CURRENT_PIN));
  twigState.shoulderCurrent = downsize10Bit(analogRead(SHOULDER_CURRENT_PIN));
  twigState.wristVoltage = downsize10Bit(analogRead(WRIST_VOLTAGE_PIN));
  twigState.shoulderVoltage = downsize10Bit(analogRead(SHOULDER_VOLTAGE_PIN));

  EncoderThreadValues encoderValues;
  if (encoderThread.getValues(encoderValues))
  {
    twigState.wristPosition = encoderValues.wrist.angle;
    twigState.wristVelocity = encoderValues.wrist.velocity;
    twigState.wristEncoderMagnitude = downsize12Bit(encoderValues.wrist.magnitude);

    twigState.shoulderPosition = encoderValues.shoulder.angle;
    twigState.shoulderVelocity = encoderValues.shoulder.velocity;
    twigState.shoulderEncoderMagnitude = downsize12Bit(encoderValues.shoulder.magnitude);

    twigState.gripperPosition = encoderValues.gripper.angle;
    twigState.gripperVelocity = encoderValues.gripper.velocity;
    twigState.gripperEncoderMagnitude = downsize12Bit(encoderValues.gripper.magnitude);
  }

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

class MainThread : public Thread
{
public: 
  void run() override
  {
    readState();
    updateLog();
    updateConnectionTimer();
    writeCommand();
    wdt_reset();
  }

  MainThread() : Thread("Main Thread", 2)
  {
  }
};

MainThread mainThread;


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

  // Runs the thread jobs directly in the main loop
  // until the unstable threading is resolved

  // encoderThread.begin();
  encoderThread.run();

  // Wait 100ms after startup to allow driver sync
  readState();
  // mainThread.start();
}

void loop()
{
  // Runs the thread jobs directly in the main loop
  // until the unstable threading is resolved
  mainThread.run();
  encoderThread.run();
}