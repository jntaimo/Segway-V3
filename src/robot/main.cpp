#include <Arduino.h>
#include "display.h"
#include "drive.h"
#include "imu.h"
#include "mic.h"
#include "pid.h"
#include "pots.h"
#include <UMS3.h>
PID pid(1, 1, 1, 0);
#define MOTOR_EN 38
UMS3 ums3;

void setup(){
    pinMode(MOTOR_EN, INPUT_PULLUP);
    Serial.begin(115200);
    ums3.begin();
    ums3.setLDO2Power(true);
    imuSetup();
    driveSetup();
    potsSetup();

}

void loop(){
    //if (!digitalRead(MOTOR_EN)){
    drive(-1, 1);
    delay(1000);
    drive(0,0);
    delay(1000);
    drive(1, -1);
    delay(1000);
    drive(-1, -1);
    delay(1000);
    drive(1,1);
    delay(1000);
    //}

}