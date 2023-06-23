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
    driveSetup();
    potsSetup();
}

void loop(){
    readPots();
    drive(potReadings[0]/1023.0, potReadings[1]/1023.0);
    Serial.printf("Pot: %d\n", potReadings[0]);
    delay(100);
}