#include <Arduino.h>
#include "PID.h"
#include "EncoderVelocity.h"
#include "display.h"
#include "drive.h"
#include "imu.h"
#include "pid.h"
#include "pots.h"
#include "util.h"
#include <UMS3.h>

#define MOTOR_EN 38
UMS3 ums3;
// Pin definitions and counts per revolution
#define LEFT_ENCODER_A_PIN 10
#define LEFT_ENCODER_B_PIN 11
#define RIGHT_ENCODER_A_PIN 13
#define RIGHT_ENCODER_B_PIN 14
#define COUNTS_PER_REVOLUTION 103.8
#define WHEEL_SEPARATION 0.4 // Wheel separation in meters
#define WHEEL_RADIUS 0.06 // Wheel radius in meters

#define MIN_KP 0.0
#define MAX_KP 10.0

#define MIN_KI 0.0
#define MAX_KI 100.0

#define MIN_KD 0.0
#define MAX_KD 10.0

#define MIN_TRIM -1.0
#define MAX_TRIM 1.0

#define PRINT_DELAY 30 // Delay between printing to serial in milliseconds
static unsigned long lastPrintTime = 0;

double balanceKp = 1.0; 
double balanceKi = 0.1;
double balanceKd = 0.01;
double balanceTrim = 0.0;
double balanceTau = 0.01;

double motorKp = 1.0;
double motorKi = 0.1;
double motorKd = 0;

// PID controllers
PID balancePID(balanceKp, balanceKi, balanceKd, balanceTau, 0.0, false);  
PID leftMotorPID(motorKp, motorKi, motorKd, 0.0, 0.0, false); 
PID rightMotorPID(motorKp, motorKi, motorKd, 0.0, 0.0, false); 

// Encoder velocity readers
EncoderVelocity leftEncoder(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN, COUNTS_PER_REVOLUTION);
EncoderVelocity rightEncoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN, COUNTS_PER_REVOLUTION);

EulerAngles imuAngles; // IMU angles


//updates the PID parameters based on the potentiometer readings
//returns true if the PID parameters have changed
//take in desired PID parameters which are changed by the function
bool updatePIDParams(double &kp, double &ki, double &kd, double &trim){
    //check the potentiometer readings
    bool newReading = readPots();
    if (newReading) {
        kp = logMap(potReadings[0], 0, 1023, MIN_KP, MAX_KP);
        ki = logMap(potReadings[1], 0, 1023, MIN_KI, MAX_KI);
        kd = logMap(potReadings[2], 0, 1023, MIN_KD, MAX_KD);
        balanceTrim = logMap(potReadings[3], 0, 1023, MIN_TRIM, MAX_TRIM);
    }
    return newReading;
}

void setup() {
  // Setup code here, such as initializing serial communication or motor drivers
    pinMode(MOTOR_EN, INPUT_PULLUP);
    Serial.begin(115200);
    ums3.begin();
    ums3.setLDO2Power(true);
    imuSetup();
    driveSetup();
    potsSetup();  
}

void loop() {
    // Read the pots
  readPots();
  // Desired values
  float desiredTiltAngle = 0; // Implement this function based on your requirements
  float desiredCurvature = 0; // Implement this function based on your requirements
  float desiredForwardVelocity = 0; // Implement this function based on your requirements

  // Read the current angle from IMU
  imuAngles = readIMU();
  //only update PID if the IMU was read successfully
  if (!imuAngles.success){
    //if the IMU was not read successfully, print an error message and return
    Serial.println("IMU read failed");
    return;
  }
  //IMU must have been read successfully
  float currentTiltAngle = imuAngles.roll;

  // Balancing control
  float balanceControl = balancePID.calculateParallel(desiredTiltAngle, currentTiltAngle);

  // Desired wheel speeds combining balance control, forward velocity, and curvature
  float leftDesiredSpeed = desiredForwardVelocity - WHEEL_SEPARATION * desiredCurvature * desiredForwardVelocity / 2 - balanceControl;
  float rightDesiredSpeed = desiredForwardVelocity + WHEEL_SEPARATION * desiredCurvature * desiredForwardVelocity / 2 + balanceControl;

  // Read current wheel speeds
  float leftCurrentSpeed = leftEncoder.getVelocity();
  float rightCurrentSpeed = rightEncoder.getVelocity();

  // Calculate motor control signals using PID controllers
  float leftMotorControl = leftMotorPID.calculateParallel(leftDesiredSpeed, leftCurrentSpeed);
  float rightMotorControl = rightMotorPID.calculateParallel(rightDesiredSpeed, rightCurrentSpeed);

  // Set motor speeds
  if (!digitalRead(MOTOR_EN)){
    drive(leftMotorControl, rightMotorControl);
  } else {
    //turn off motors if motor enable is not set
    drive(0,0);
  }
  
  // Print to serial
  
  if (millis() - lastPrintTime > PRINT_DELAY) {
    Serial.print("Desired tilt angle: ");
    Serial.print(desiredTiltAngle);
    Serial.print(" Current tilt angle: ");
    Serial.print(currentTiltAngle);
    Serial.print(" Balance control: ");
    Serial.print(balanceControl);
    Serial.print(" Left desired speed: ");
    Serial.print(leftDesiredSpeed);
    Serial.print(" Right desired speed: ");
    Serial.print(rightDesiredSpeed);
    Serial.print(" Left current speed: ");
    Serial.print(leftCurrentSpeed);
    Serial.print(" Right current speed: ");
    Serial.print(rightCurrentSpeed);
    Serial.print(" Left motor control: ");
    Serial.print(leftMotorControl);
    Serial.print(" Right motor control: ");
    Serial.println(rightMotorControl);
    lastPrintTime = millis();
  }
  
  // Delay to avoid overloading the microcontroller
  delay(1);
}
