#include <Arduino.h>
#include "drive.h"

//pins wired to the motor controller
#define DIR1 39
#define DIR2 40
#define PWM1 41
#define PWM2 42

//increase frequency of pwm so it can't be heard
#define PWM_FREQ 25000
//max number of bits for pwm
#define PWM_BITS 11
#define MAX_PWM (pow(2, PWM_BITS)-1)
#define PWM1_CHANNEL 0
#define PWM2_CHANNEL 1

float batteryVoltage = 12;

void driveSetup(){
    //configure pins as outputs
    pinMode(DIR1, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(PWM2, OUTPUT);

    //configure pwm channels
    ledcSetup(0, PWM_FREQ, PWM_BITS);
    ledcSetup(1, PWM_FREQ, PWM_BITS);

    //assign pwm pins to channels
    ledcAttachPin(PWM1, 0);
    ledcAttachPin(PWM2, 1);
}

//drives the motors with units in volts
void driveVolts(double left, double right){
    drive(left/batteryVoltage, right/batteryVoltage);
}

//drive motors with values from -1 to 1 
//positive values move the motor forward
void drive(double left, double right){
    //limit drive pwm 
    left = constrain(left, -1, 1);
    right = constrain(right, -1, 1);
    //pick direction
    digitalWrite(DIR1, left > 0);
    digitalWrite(DIR2, right > 0);

    //output pwm
    ledcWrite(PWM1_CHANNEL, abs(left)*MAX_PWM);
    ledcWrite(PWM2_CHANNEL, abs(right)*MAX_PWM);
}
