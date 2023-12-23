#include <Arduino.h>
#include "drive.h"
#include "pinout.h"
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

    // //assign pwm pins to channels
    ledcAttachPin(PWM1, L_CHANNEL);
    ledcAttachPin(PWM2, R_CHANNEL); 
    
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
    ledcWrite(L_CHANNEL, abs(left)*MAX_PWM);
    ledcWrite(R_CHANNEL, abs(right)*MAX_PWM);
}

void setup(){
    driveSetup();
    Serial.begin(115200);
}

void loop(){
    Serial.println("Forward");
    drive(1, 1);
    delay(500);
    Serial.println("Stop");
    drive(0,0);
    delay(500);
    Serial.println("Backward");
    drive(-1, -1);
    delay(500);
    Serial.println("Stop");
    drive(0,0);
    delay(500);
}
