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
#define PWM_BITS 10
#define MAX_PWM (pow(2, PWM_BITS)-1)
#define L_CHANNEL 0
#define R_CHANNEL 1

float batteryVoltage = 12;

void driveSetup(){
    //configure pins as outputs
    pinMode(DIR1, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(PWM2, OUTPUT);

    //configure pwm channels
    ledcSetup(L_CHANNEL, PWM_FREQ, PWM_BITS);
    ledcSetup(R_CHANNEL, PWM_FREQ, PWM_BITS);

    //assign pwm pins to channels
    ledcAttachPin(DIR1, 0);
    ledcAttachPin(DIR2, 1);
    
    //put pwm at middle value to turn off motors
    ledcWrite(L_CHANNEL, MAX_PWM/2);
    ledcWrite(R_CHANNEL, MAX_PWM/2);

    //force motor driver into two wire mode
    digitalWrite(PWM1, HIGH);
    digitalWrite(PWM2, HIGH);
    

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

    //output pwm
    //centered around half of max pwm
    ledcWrite(L_CHANNEL, left*MAX_PWM/2 + MAX_PWM/2);
    ledcWrite(R_CHANNEL, right*MAX_PWM/2 + MAX_PWM/2);
}

