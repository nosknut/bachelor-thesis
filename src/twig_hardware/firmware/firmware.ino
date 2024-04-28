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

  encoderThread.shoulderEncoder.minMagnitude = config.shoulderEncoderMinMagnitude;
  encoderThread.wristEncoder.minMagnitude = config.wristEncoderMinMagnitude;
  encoderThread.gripperEncoder.minMagnitude = config.gripperEncoderMinMagnitude;

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


  EncoderThreadValues encoderValues;
  if (encoderThread.getValues(encoderValues))
  {
    twigState.wristPosition = encoderValues.wrist.angle;
    twigState.wristVelocity = encoderValues.wrist.velocity;
    twigState.wristEncoderMagnitude = encoderValues.wrist.magnitude;

    twigState.shoulderPosition = encoderValues.shoulder.angle;
    twigState.shoulderVelocity = encoderValues.shoulder.velocity;
    twigState.shoulderEncoderMagnitude = encoderValues.shoulder.magnitude;

    twigState.gripperPosition = encoderValues.gripper.angle;
    twigState.gripperVelocity = encoderValues.gripper.velocity;
    twigState.gripperEncoderMagnitude = encoderValues.gripper.magnitude;
  }

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
  void run() override
  {
    readState();
    updateLog();
    updateConnectionTimer();
    writeCommand();
    wdt_reset();
    delay(10);
  }

public: 
  MainThread() : Thread("Main Thread")
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

  encoderThread.begin();

  // Wait 100ms after startup to allow driver sync
  readState();
  delay(100);

  mainThread.start();
}

void loop()
{
}