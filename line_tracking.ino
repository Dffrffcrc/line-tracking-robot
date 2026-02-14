#include "CytronMotorDriver.h"

// Motor driver setup
CytronMD motor1(PWM_PWM, 3, 9);   // PWM 1A = Pin 3, PWM 1B = Pin 9.
CytronMD motor2(PWM_PWM, 10, 11); // PWM 2A = Pin 10, PWM 2B = Pin 11.

// IR Sensor pins
int ir1Pin = 2;  
int ir2Pin = 4;  

void setup() {
  Serial.begin(115200);
  
  pinMode(ir1Pin, INPUT);  // Right sensor 
  pinMode(ir2Pin, INPUT);  // Left sensor 
}

void loop() {
  int rightVal = digitalRead(ir1Pin); 
  int leftVal = digitalRead(ir2Pin);   
  
  if (leftVal == 0 && rightVal == 0) {
    forward();
  }
  else if (leftVal == 0 && rightVal == 1) {
    turnRight();
  }
  else if (leftVal == 1 && rightVal == 0) {
    turnLeft();
  }
  else if (leftVal == 1 && rightVal == 1) {
    Stop();
  }

  delay(100);


  
  // Debug output
  Serial.print("Left: ");
  Serial.print(leftVal);
  Serial.print(" | Right: ");
  Serial.println(rightVal);
  
}

void forward() {
  motor1.setSpeed(128); 
  motor2.setSpeed(128);
}

void turnLeft() {
  motor1.setSpeed(-128);   
  motor2.setSpeed(128);   
}

void turnRight() {
  motor1.setSpeed(128);   
  motor2.setSpeed(-128);   
}

void Stop() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}