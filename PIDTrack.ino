#include "CytronMotorDriver.h"

CytronMD motor1(PWM_PWM, 3, 9);   // PWM 1A = Pin 3, PWM 1B = Pin 9.
CytronMD motor2(PWM_PWM, 10, 11); // PWM 2A = Pin 10, PWM 2B = Pin 11.

int ir1Pin = 4;  // Right sensor 
int ir2Pin = 2;  // Left sensor 

float Kp = 50.0;      
float Ki = 0.5;       
float Kd = 20.0;      

float lastError = 0;  
float integral = 0;   
int baseSpeed = 200;  
int maxSpeed = 255;   

float integralMax = 100.0;  

unsigned long lastTime = 0;
float dt = 0.01;  

void setup() {
  Serial.begin(115200);
  
  pinMode(ir1Pin, INPUT);  // Right sensor 
  pinMode(ir2Pin, INPUT);  // Left sensor 
  
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;  
  lastTime = currentTime;
  
  int rightVal = digitalRead(ir1Pin); 
  int leftVal = digitalRead(ir2Pin);   
  
  float error = 0;
  
  if (leftVal == 0 && rightVal == 0) {
    error = 0;
  }
  else if (leftVal == 0 && rightVal == 1) {
    error = 1;
  }
  else if (leftVal == 1 && rightVal == 0) {
    error = -1;
  }
  else if (leftVal == 1 && rightVal == 1) {
    error = lastError;
  }
  
  if (!(leftVal == 1 && rightVal == 1)) {
    integral += error * dt; 
    integral = constrain(integral, -integralMax, integralMax);
  }

  float derivative = 0;
  if (dt > 0) {
    derivative = (error - lastError) / dt;
  }
  
  float proportional = Kp * error;
  float integralTerm = Ki * integral;
  float derivativeTerm = Kd * derivative;
  
  float correction = proportional + integralTerm + derivativeTerm;
  
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
  Serial.print(error, 2);
  Serial.print(" | P: ");
  Serial.print(proportional, 2);
  Serial.print(" | I: ");
  Serial.print(integralTerm, 2);
  Serial.print(" | D: ");
  Serial.print(derivativeTerm, 2);
  Serial.print(" | Corr: ");
  Serial.print(correction, 2);
  Serial.print(" | LS: ");
  Serial.print(leftSpeed);
  Serial.print(" | RS: ");
  Serial.println(rightSpeed);
  
  delay(10); 
}