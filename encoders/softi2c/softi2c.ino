// https://github.com/Seeed-Studio/Seeed_Arduino_AS5600/blob/master/examples/readAngle/readAngle.ino
#include <Wire.h>
#include <SoftI2C.h>
#include "AS5600.h"

SoftI2C softWire1 = SoftI2C(4, 5); // sda, scl
SoftI2C softWire2 = SoftI2C(6, 5); // sda, scl

AS5600 encoder1(&softWire1);
AS5600 encoder2(&softWire2);

void setup()
{
  Serial.begin(115200);
  softWire1.begin();
  softWire2.begin();
}

float rawToDeg(float rawAngle)
{
  return rawAngle * (360.0 / 4095.0);
}

float getAngle(AS5600 &encoder)
{
  return rawToDeg(encoder.readAngle());
}

void leadingZeros(String num, int length)
{
  int numLen = num.length();
  for (int i = 0; i < length - numLen; i++)
  {
    Serial.print(" ");
  }
  Serial.print(num);
}

void printEncoder(AS5600 &encoder, int encoderNum)
{
  Serial.print("A");
  Serial.print(encoderNum);
  Serial.print(": ");
  leadingZeros(String(getAngle(encoder)), 6);
  Serial.print(", ");
  Serial.print("M");
  Serial.print(encoderNum);
  Serial.print(": ");
  leadingZeros(String(encoder.readMagnitude()), 4);
}

void loop()
{
  printEncoder(encoder1, 1);
  Serial.print(", ");
  printEncoder(encoder2, 2);
  Serial.print(", ");

  auto timer = micros();
  encoder1.readAngle();
  encoder2.readAngle();
  Serial.print(micros() - timer);

  Serial.println();
}