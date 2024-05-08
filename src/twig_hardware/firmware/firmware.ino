#include <Servo.h>

Servo s;
Servo w;
Servo g;

void setup() {
  s.attach(3);
  w.attach(6);
  g.attach(8);

  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
}

void loop() { 
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  s.writeMicroseconds(1500);
  w.writeMicroseconds(1500);
  g.writeMicroseconds(1500);
  delay(1000);
  s.writeMicroseconds(1600);
  w.writeMicroseconds(1500);
  delay(1000);
  s.writeMicroseconds(1500);
  g.writeMicroseconds(1600);
  delay(1000);
  g.writeMicroseconds(1500);
  w.writeMicroseconds(1600);
  delay(1000);
}