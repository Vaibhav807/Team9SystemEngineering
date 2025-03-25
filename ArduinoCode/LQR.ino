#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const int enA = 9;
const int enB = 10;
const int in1 = 6;
const int in2 = 7;
const int in3 = 4;
const int in4 = 5;

const int leftEncA  = 2;
const int leftEncB  = 3;
const int rightEncA = 12;
const int rightEncB = 13;

volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;

Adafruit_MPU6050 mpu;

float x = 0;
float x_dot = 0;
float theta = 0;
float theta_dot = 0;

float dt;
unsigned long lastTime;

float K1 = -40;
float K2 = -10;
float K3 = -145;
float K4 = -15;

void leftEncoderISR() {
  if (digitalRead(in2_RL) == LOW) leftEncoderCount--;
  else leftEncoderCount++;
}

void rightEncoderISR() {
  if (digitalRead(in2_RR) == LOW) rightEncoderCount--;
  else rightEncoderCount++;
}

void setup() {
  Serial.begin(115200);
  mpu.begin();
  pinMode(enA_FL, OUTPUT);
  pinMode(in1_FL, OUTPUT);
  pinMode(in2_FL, OUTPUT);
  pinMode(enA_FR, OUTPUT);
  pinMode(in1_FR, OUTPUT);
  pinMode(in2_FR, OUTPUT);
  pinMode(enA_RL, OUTPUT);
  pinMode(in1_RL, OUTPUT);
  pinMode(in2_RL, OUTPUT);
  pinMode(enA_RR, OUTPUT);
  pinMode(in1_RR, OUTPUT);
  pinMode(in2_RR, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(in1_RL), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(in1_RR), rightEncoderISR, RISING);
  lastTime = micros();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  lastTime = now;
  theta = atan2(a.acceleration.y, a.acceleration.z);
  theta_dot = g.gyro.x;
  x = leftEncoderCount * 0.001;
  x_dot = (float)(leftEncoderCount - rightEncoderCount) * 0.0001;
  float u = -(K1 * x + K2 * x_dot + K3 * theta + K4 * theta_dot);
  if (u > 0) {
    digitalWrite(in1_FL, HIGH); digitalWrite(in2_FL, LOW);
    digitalWrite(in1_FR, HIGH); digitalWrite(in2_FR, LOW);
    digitalWrite(in1_RL, HIGH); digitalWrite(in2_RL, LOW);
    digitalWrite(in1_RR, HIGH); digitalWrite(in2_RR, LOW);
  } else {
    digitalWrite(in1_FL, LOW); digitalWrite(in2_FL, HIGH);
    digitalWrite(in1_FR, LOW); digitalWrite(in2_FR, HIGH);
    digitalWrite(in1_RL, LOW); digitalWrite(in2_RL, HIGH);
    digitalWrite(in1_RR, LOW); digitalWrite(in2_RR, HIGH);
  }
  int speed = (int)constrain(fabs(u), 0, 255);
  analogWrite(enA_FL, speed);
  analogWrite(enA_FR, speed);
  analogWrite(enA_RL, speed);
  analogWrite(enA_RR, speed);
  Serial.print(x); Serial.print(",");
  Serial.print(x_dot); Serial.print(",");
  Serial.print(theta); Serial.print(",");
  Serial.print(theta_dot); Serial.print(",");
  Serial.println(u);
  delay(10);
}
