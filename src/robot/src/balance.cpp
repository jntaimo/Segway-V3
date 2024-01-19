#include <Arduino.h>
#include "PID.h"
#include "EncoderVelocity.h"
#include "display.h"
#include "drive.h"
#include "imu.h"
#include "pid.h"
#include "pots.h"
#include "util.h"
#include "wireless.h"
#include "balance.h"
#include "pinout.h"
#include <UMS3.h>


UMS3 ums3;
// Pin definitions and counts per revolution

static unsigned long lastPrintTime = 0;

double balanceKp = 10.0; 
double balanceKi = 0.1;
double balanceKd = 0.00;
double balanceTrim = 0.0;
double balanceTau = 0.05;

double motorKp = 1.0;
double motorKi = 0.1;
double motorKd = 0.0;

double observerKp = 1.0;
double observerKi = 0.1;
double observerKd = 0.0;

// PID controllers
PID balancePID(balanceKp, balanceKi, balanceKd, 0, balanceTau, false);  
PID leftMotorPID(motorKp, motorKi, motorKd, 0.0, 0.0, false); 
PID rightMotorPID(motorKp, motorKi, motorKd, 0.0, 0.0, false); 

// Encoder velocity readers
EncoderVelocity leftEncoder(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN, COUNTS_PER_REVOLUTION);
EncoderVelocity rightEncoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN, COUNTS_PER_REVOLUTION);

EulerAngles imuAngles; // IMU angles
EulerAngles imuGyro; // IMU angles from gyro

//updates the PID parameters based on the potentiometer readings
//returns true if the PID parameters have changed
//take in desired PID parameters which are changed by the function
bool updatePIDParams(double &kp, double &ki, double &kd, double &trim){
    //check the potentiometer readings
    bool newReading = readPots();
    if (newReading) {
        kp = mapDouble(potReadings[2], 1023, 0, MIN_KP, MAX_KP);
        ki = mapDouble(potReadings[1], 1023, 0, MIN_KI, MAX_KI);
        kd = mapDouble(potReadings[0], 1023, 0, MIN_KD, MAX_KD);
        balanceTrim = mapDouble(potReadings[3], 1023, 0, MIN_TRIM, MAX_TRIM);
    }
    return newReading;
}

void setup() {
  // Setup code here, such as initializing serial communication or motor drivers
    Serial.begin(115200);
    ums3.begin();
    ums3.setLDO2Power(false);
    imuSetup();
    driveSetup();
    potsSetup(); 
    wirelessSetup();
    // displaySetup();
    balancePID.setParallelTunings(balanceKp, balanceKi, balanceKd, balanceTau, -0.01, 0.01);
}

unsigned long lastLoopTime = 0;
unsigned long loopDuration = 1;

float desiredTiltAngle = 0; 
float desiredCurvature = 0; 
float desiredForwardVelocity = 0; 

float leftMotorAngle = 0;
float rightMotorAngle = 0;

void loop() {
  loopDuration = micros() - lastLoopTime;
  lastLoopTime = micros();

  //if a new message has been received, update the desired tilt angle
  //low pass filter the desired tilt angle to avoid a step response
  if (true){
    freshWirelessData = false;
    //normalize x and y from -1 to 1
    float x = mapDouble(joystick.x, 0, 1023, -1, 1);
    float y = mapDouble(joystick.y, 0, 1023, -1, 1);
    //deadband
    if (abs(x) < CONTROLLER_DEADBAND){
      x = 0;
    }
    if (abs(y) < CONTROLLER_DEADBAND){
      y = 0;
    }
    //TODO Try FIR filter to maintin responsiveness

    //convert the joystick x and y to tilt angle and curvature
    float newTiltAngle = y*MAX_CONTROLLER_TILT*PI/180.0;
    //calculate alpha for the low pass filter based on the loop duration
    float alpha = loopDuration/(loopDuration + CONTROLLER_TAU*1000000);

    desiredTiltAngle = desiredTiltAngle*alpha + (1-alpha)*newTiltAngle;
    desiredCurvature = desiredCurvature*alpha + (1-alpha)*(x/4);
    freshWirelessData = false;
  }

  // Read the current angle from IMU
  EulerAngles newIMUAngles = readIMU();

  //only update PID if the IMU was read successfully
  if (!newIMUAngles.success){
    //if the IMU was not read successfully, print an error message and return
    Serial.println("IMU read failed");
    return;
  }
  //got an angle reading from the IMU
  if (!newIMUAngles.gyro){
    imuAngles = newIMUAngles;
  } else {
    //if the reading was from the gyro
    imuGyro = newIMUAngles;
  }

  //update the PID parameters
  bool paramsChanged = updatePIDParams(balanceKp, balanceKi, balanceKd, balanceTrim);
  if (paramsChanged) {
    balancePID.setParallelTunings(balanceKp, balanceKi, balanceKd);
    // updateDisplay(balanceKp, balanceKi, balanceKd, balanceTrim);
  }
  
  //IMU must have been read successfully
  float currentTiltAngle = imuAngles.roll;

  // Balancing control
  float balanceControl = balancePID.calculateParallel(desiredTiltAngle + balanceTrim, currentTiltAngle, imuGyro.roll);

  // // Desired wheel speeds combining balance control, forward velocity, and curvature
  // float leftDesiredSpeed = desiredForwardVelocity - WHEEL_SEPARATION * desiredCurvature * desiredForwardVelocity / 2 - balanceControl;
  // float rightDesiredSpeed = desiredForwardVelocity + WHEEL_SEPARATION * desiredCurvature * desiredForwardVelocity / 2 + balanceControl;

  // // Read current wheel speeds
  // float leftCurrentSpeed = leftEncoder.getVelocity();
  // float rightCurrentSpeed = rightEncoder.getVelocity();

  // // Calculate motor control signals using PID controllers
  // float leftMotorControl = leftMotorPID.calculateParallel(leftDesiredSpeed, leftCurrentSpeed);
  // float rightMotorControl = rightMotorPID.calculateParallel(rightDesiredSpeed, rightCurrentSpeed);
  // leftMotorAngle = leftEncoder.getPosition();
  // rightMotorAngle = rightEncoder.getPosition();

    // Set motor speeds
  if (!isnan(balanceControl) && abs((currentTiltAngle + balanceTrim)*180/PI) < MAX_TILT){
    drive(balanceControl + desiredCurvature, balanceControl - desiredCurvature);
  } else {
    //turn off motors if motor enable is not set
    drive(0,0);
  }
  
  // Print to serial
  
  if (millis() - lastPrintTime > PRINT_DELAY) {
    Serial.print("Desired tilt angle: ");
    Serial.print(desiredTiltAngle*180.0/PI);
    Serial.print(" Tilt angle: ");
    Serial.print(currentTiltAngle*180.0/PI);
    Serial.print(" Control Signal: ");
    Serial.print(balanceControl);

    //print the PID parameters
    Serial.print(" Kp: ");
    Serial.print(balanceKp);
    Serial.print(" Ki: ");
    Serial.print(balanceKi);
    Serial.print(" Kd: ");
    Serial.print(balanceKd);
    Serial.print(" Trim: ");
    Serial.println(balanceTrim);
    // Serial.print("L Angle: ");
    // Serial.print(int(leftMotorAngle*180.0/PI));
    // Serial.print("° R Angle: ");
    // Serial.print(int(rightMotorAngle*180.0/PI));
    // Serial.println("°");
    // Serial.print(" Loop Time: ");
    // Serial.println(loopDuration);


    // Serial.print(" Left desired speed: ");
    // Serial.print(leftDesiredSpeed);
    // Serial.print(" Right desired speed: ");
    // Serial.print(rightDesiredSpeed);
    // Serial.print(" Left current speed: ");
    // Serial.print(leftCurrentSpeed);
    // Serial.print(" Right current speed: ");
    // Serial.print(rightCurrentSpeed);
    // Serial.print(" Left motor control: ");
    // Serial.print(leftMotorControl);
    // Serial.print(" Right motor control: ");
    // Serial.println(rightMotorControl);
    lastPrintTime = millis();
  }
  
}
