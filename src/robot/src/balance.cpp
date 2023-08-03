#include <Arduino.h>
#include "PID.h"
#include "EncoderVelocity.h"
#include "display.h"
#include "drive.h"
#include "imu.h"
#include "pid.h"
#include "pots.h"
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

//radi
#define MIN_TRIM -1.0
#define MAX_TRIM 1.0

double balanceKp = 1.0; 
double balanceKi = 0.1;
double balanceKd = 0.01;
double balanceTrim = 0.0;

double motorKp = 1.0;
double motorKi = 0.1;
double motorKd = 0;

// PID controllers
PID balancePID(1.0, 0.1, 0.01, 0.0, 0.0, true);  // Adjust these parameters
PID leftMotorPID(1.0, 0.1, 0.01, 0.0, 0.0, true); // Adjust these parameters
PID rightMotorPID(1.0, 0.1, 0.01, 0.0, 0.0, true); // Adjust these parameters

// Encoder velocity readers
EncoderVelocity leftEncoder(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN, COUNTS_PER_REVOLUTION);
EncoderVelocity rightEncoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN, COUNTS_PER_REVOLUTION);

EulerAngles imuAngles; // IMU angles

double logMap(double x, double in_min, double in_max, double out_min, double out_max);
//updates the PID parameters based on the potentiometer readings
//returns true if the PID parameters have changed
bool updatePIDParams(){
    //check the potentiometer readings
    bool newReading = readPots();
    if (newReading) {
        balanceKp = logMap(potReadings[0], 0, 1023, MIN_KP, MAX_KP);
        balanceKi = logMap(potReadings[1], 0, 1023, MIN_KI, MAX_KI);
        balanceKd = logMap(potReadings[2], 0, 1023, MIN_KD, MAX_KD);
        //map the potentiometer reading to a value between MIN_KP and MAX_KP logarithmically
        
    }
}

//map from an input to output range logarithmically
double logMap(double x, double in_min, double in_max, double out_min, double out_max) {
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;

  // Normalize the input value to the [0, 1] range
  double normalized = (log(x) - log(in_min)) / (log(in_max) - log(in_min));

  // Map the normalized value to the output range
  return normalized * (out_max - out_min) + out_min;
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
  float currentTiltAngle = imuAngles.roll;

  // Balancing control
  float balanceControl = balancePID.calculateSerial(desiredTiltAngle, currentTiltAngle);

  // Desired wheel speeds combining balance control, forward velocity, and curvature
  float leftDesiredSpeed = desiredForwardVelocity - WHEEL_SEPARATION * desiredCurvature * desiredForwardVelocity / 2 - balanceControl;
  float rightDesiredSpeed = desiredForwardVelocity + WHEEL_SEPARATION * desiredCurvature * desiredForwardVelocity / 2 + balanceControl;

  // Read current wheel speeds
  float leftCurrentSpeed = leftEncoder.getVelocity();
  float rightCurrentSpeed = rightEncoder.getVelocity();

  // Calculate motor control signals using PID controllers
  float leftMotorControl = leftMotorPID.calculateSerial(leftDesiredSpeed, leftCurrentSpeed);
  float rightMotorControl = rightMotorPID.calculateSerial(rightDesiredSpeed, rightCurrentSpeed);

  // Set motor speeds
  if (!digitalRead(MOTOR_EN)){
    drive(leftMotorControl, rightMotorControl);
  } else {
    //turn off motors if motor enable is not set
    drive(0,0);
  }
  

  // Delay to avoid overloading the microcontroller
  delay(1);
}
