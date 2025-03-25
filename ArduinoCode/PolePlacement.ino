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

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

Adafruit_MPU6050 mpu;

float kAngle = 50.0;
float kRate  = 1.0;

float targetAngle = 0.0;
float output;
float maxOutput = 255;

float angleX = 0;
float gyroXAngle = 0;
float compAngleX = 0;
float alpha = 0.96;
unsigned long timer = 0;
float dt;

const int serialPrintInterval = 50;
unsigned long lastPrintTime = 0;

void leftEncoderISR() {
  if (digitalRead(leftEncB) == LOW) {
    leftEncoderCount--;
  } else {
    leftEncoderCount++;
  }
}

void rightEncoderISR() {
  if (digitalRead(rightEncB) == LOW) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(leftEncA, INPUT_PULLUP);
  pinMode(leftEncB, INPUT_PULLUP);
  pinMode(rightEncA, INPUT_PULLUP);
  pinMode(rightEncB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncA), rightEncoderISR, RISING);

  Serial.println("System ready - Pole Placement Balancing with encoders");
  Serial.println("Time(ms),Angle(deg),AngleRate,ControlOut,LeftEnc,RightEnc");

  timer = micros();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  dt = (micros() - timer) / 1000000.0;
  timer = micros();

  angleX = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  gyroXAngle += g.gyro.x * dt;
  compAngleX = alpha * (compAngleX + g.gyro.x * dt) + (1 - alpha) * angleX;

  float x1 = compAngleX - targetAngle;
  float x2 = g.gyro.x * (180.0 / PI);

  output = -(kAngle * x1 + kRate * x2);

  output = constrain(output, -maxOutput, maxOutput);

  if (output > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, abs(output));
    analogWrite(enB, abs(output));
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, abs(output));
    analogWrite(enB, abs(output));
  }

  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= serialPrintInterval) {
    lastPrintTime = currentTime;

    Serial.print(currentTime);
    Serial.print(",");
    Serial.print(compAngleX);
    Serial.print(",");
    Serial.print(x2);
    Serial.print(",");
    Serial.print(output);
    Serial.print(",");
    Serial.print(leftEncoderCount);
    Serial.print(",");
    Serial.println(rightEncoderCount);

    Serial.print("Time: ");
    Serial.print(currentTime / 1000.0, 2);
    Serial.print("s | Angle: ");
    Serial.print(compAngleX, 2);
    Serial.print("° | AngleRate: ");
    Serial.print(x2, 2);
    Serial.print(" deg/s | ControlOut: ");
    Serial.print(output, 0);
    Serial.print(" | Enc L/R: ");
    Serial.print(leftEncoderCount);
    Serial.print("/");
    Serial.println(rightEncoderCount);
  }

  delay(2);
}
