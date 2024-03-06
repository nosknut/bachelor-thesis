#include <Servo.h>

// 720 for NANO, 1024 for UNO
const int MAX_ANALOG_IN = 720;
byte SERVO_PIN = 9;
int SERVO_CENTER = 1500;
// For some reason speed does not change after 50% of max travel
int SERVO_TRAVEL = 0.5 * 600;
float DEADBAND = 0.05;
byte POTMETER_PIN = A0;

Servo servo;

void setup() {
  Serial.begin(115200);
  servo.attach(SERVO_PIN);
  
  while (readPotmeter() > 0.05) {
    Serial.println("Zero the potmeter to continue: " + String(readPotmeter() * 100) + "%");
  }
}

float readPotmeter() {
  float potmeterSpeed = ((analogRead(POTMETER_PIN) / float(MAX_ANALOG_IN)) - 0.5) * 2;

  if (abs(potmeterSpeed) > DEADBAND) {
    return potmeterSpeed;
  }

  return 0;
}

int percentageSpeedToMicros(float servoSpeed)
{
    // Range: 900 to 2100 = 1500 +- 600
    int microseconds = SERVO_CENTER + (servoSpeed * SERVO_TRAVEL);
    return microseconds;
}

void loop() {
  float potmeterSpeed= readPotmeter();
  int microseconds = percentageSpeedToMicros(potmeterSpeed);
  servo.writeMicroseconds(microseconds);
  
  Serial.print("Speed: " + String(potmeterSpeed * 100) + "%");
  Serial.print("  Microseconds: " + String(microseconds) + "ms");
  Serial.println("");
}
