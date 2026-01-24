#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

// --- PIN DEFINITIONS (MATCHING YOUR L298N WIRING) ---
// Right Motor
#define ENA 14
#define IN1 26
#define IN2 27

// Left Motor
#define IN3 25
#define IN4 33
#define ENB 32

// --- PID CONSTANTS (THE TUNING KNOBS) ---
// You will change these later to make it balance perfectly!
float Kp = 5.0; 
float Kd = 0.5;
float Ki = 0.0;

// Variables
float targetAngle = 0.0;
float currentAngle = 0.0;
float previousError = 0.0;
float integral = 0.0;
float output = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize Motor Pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  
  // Initialize Sensor
  Serial.println("Calibrating MPU6050... DO NOT MOVE ROBOT");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true); // This takes a few seconds
  Serial.println("Calibration Done!");
}

void driveMotors(float speed) {
  // Deadzone compensation (Motors won't move below ~40 power)
  int minPower = 40; 
  int pwm = abs(speed);
  
  if (pwm > 255) pwm = 255;
  if (pwm < 5) pwm = 0; // Ignore tiny noise
  else pwm += minPower; 

  // Motor Direction Logic
  if (speed > 0) { // Forward (Adjust if inverted)
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else { // Backward
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  }
  
  // Send Speed to Motors
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
}

void loop() {
  mpu6050.update();
  currentAngle = mpu6050.getAngleX(); // Check if X or Y axis depending on mounting!

  // --- SAFETY SHUTOFF ---
  // If robot falls over (> 45 degrees), turn off motors
  if (abs(currentAngle) > 45) {
    driveMotors(0);
    integral = 0; // Reset memory
    return;
  }

  // --- PID CALCULATION ---
  float error = targetAngle - currentAngle;
  integral += error;
  float derivative = error - previousError;
  
  // The Formula
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  previousError = error;

  driveMotors(output);
  
  // Debugging (View this in Serial Monitor)
  // Serial.print("Angle: "); Serial.print(currentAngle);
  // Serial.print(" Output: "); Serial.println(output);
}