#include "CytronMotorDriver.h"

CytronMD motor1(PWM_PWM, 3, 9);
CytronMD motor2(PWM_PWM, 10, 11);

int ir1Pin = 4;
int ir2Pin = 2;

float Kp = 50.0;
float Kd = 20.0;
float lastError = 0;
int baseSpeed = 100;
int maxSpeed = 255;
int turnSpeed = 180;

void setup() {
  Serial.begin(115200);
  pinMode(ir1Pin, INPUT);
  pinMode(ir2Pin, INPUT);
}

void loop() {
  lineTrackForDuration(6500);
  turnRightForDuration(600);

  motor1.setSpeed(0);
  motor2.setSpeed(0);

  Serial.println("=== MISSION COMPLETE ===");

  while (true) {
    delay(100);
  }
}

void lineTrackForDuration(unsigned long duration) {
  unsigned long startTime = millis();

  Serial.print("Starting line tracking for ");
  Serial.print(duration);
  Serial.println(" ms");

  while (millis() - startTime < duration) {
    int rightVal = digitalRead(ir1Pin);
    int leftVal = digitalRead(ir2Pin);

    float error = 0;

    if (leftVal == 0 && rightVal == 0) {
      error = 0;
    } else if (leftVal == 0 && rightVal == 1) {
      error = 1;
    } else if (leftVal == 1 && rightVal == 0) {
      error = -1;
    } else if (leftVal == 1 && rightVal == 1) {
      error = lastError;
    }

    float derivative = error - lastError;
    float correction = (Kp * error) + (Kd * derivative);

    int leftSpeed = constrain(baseSpeed + correction, -maxSpeed, maxSpeed);
    int rightSpeed = constrain(baseSpeed - correction, -maxSpeed, maxSpeed);

    motor1.setSpeed(rightSpeed);
    motor2.setSpeed(leftSpeed);

    lastError = error;

    Serial.print("L: ");
    Serial.print(leftVal);
    Serial.print(" | R: ");
    Serial.print(rightVal);
    Serial.print(" | Err: ");
    Serial.print(error);
    Serial.print(" | Corr: ");
    Serial.print(correction);
    Serial.print(" | LS: ");
    Serial.print(leftSpeed);
    Serial.print(" | RS: ");
    Serial.println(rightSpeed);

    delay(10);
  }

  Serial.println("Line tracking duration complete");
}

void turnRightForDuration(unsigned long duration) {
  Serial.print("Turning RIGHT for ");
  Serial.print(duration);
  Serial.println(" ms");

  unsigned long startTime = millis();

  motor1.setSpeed(turnSpeed);
  motor2.setSpeed(-turnSpeed);

  while (millis() - startTime < duration) {
    delay(10);
  }

  motor1.setSpeed(0);
  motor2.setSpeed(0);

  Serial.println("Turn RIGHT complete");

  delay(200);
  lastError = 0;
}