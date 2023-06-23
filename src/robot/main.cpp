#include <Arduino.h>
#include "display.h"
#include "drive.h"
#include "imu.h"
#include "mic.h"
#include "pid.h"
#include "pots.h"

void setup(){
    Serial.begin(115200);
    imuSetup();
}

void loop(){
    EulerAngles angle = readIMU();
    printEuler(angle);
    delay(100);
}