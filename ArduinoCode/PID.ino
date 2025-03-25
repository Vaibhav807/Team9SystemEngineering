#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// These are the pins including Encoder Pins
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

float Kp = 52; 
float Ki = 0.5;
float Kd = 8;
float targetAngle = 0.0;
float error, lastError = 0;
float integral = 0;
float derivative;
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
    while (1) delay(10);
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

  Serial.println("System ready - PID balancing with encoders");
  Serial.println("Time(ms),Angle(deg),Error,PID_Output,LeftEnc,RightEnc");
  timer = micros();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  dt = (micros() - timer) / 1000000.0;
  timer = micros();

  angleX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  gyroXAngle += g.gyro.x * dt;
  compAngleX = alpha * (compAngleX + g.gyro.x * dt) + (1 - alpha) * angleX;

  error = compAngleX - targetAngle;
  float P = Kp * error;
  integral += error * dt;
  integral = constrain(integral, -maxOutput/Ki, maxOutput/Ki);
  float I = Ki * integral;
  derivative = (error - lastError) / dt;
  float D = Kd * derivative;
  lastError = error;

  output = P + I + D;
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
    
    Serial.print(currentTime); Serial.print(",");
    Serial.print(compAngleX); Serial.print(",");
    Serial.print(error); Serial.print(",");
    Serial.print(output); Serial.print(",");
    Serial.print(leftEncoderCount); Serial.print(",");
    Serial.println(rightEncoderCount);
    
    Serial.print("Time: "); Serial.print(currentTime/1000.0, 2); Serial.print("s | ");
    Serial.print("Angle: "); Serial.print(compAngleX, 2); Serial.print("° | ");
    Serial.print("Error: "); Serial.print(error, 2); Serial.print(" | ");
    Serial.print("PID Out: "); Serial.print(output, 0); Serial.print(" | ");
    Serial.print("Enc L/R: "); Serial.print(leftEncoderCount); 
    Serial.print("/"); Serial.println(rightEncoderCount);
  }

  delay(2);
}
