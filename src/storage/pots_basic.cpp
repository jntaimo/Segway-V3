#include <Arduino.h>
#include "pots.h"
#include "pinout.h"

#define NUM_POTS 4
#define POT_THRESHOLD 3
const uint8_t potPins[] = {POT_1_PIN, POT_2_PIN, POT_3_PIN, POT_4_PIN};

uint16_t potReadings[4];

void potsSetup(){
    analogReadResolution(10);
    //configure the potentiometers as inputs
    for (uint8_t i: potPins){
        pinMode(i, INPUT);
    }
}

//reads all of the potentiometers and updates the potReadings array
//returns true if any of the potentiometers have changed by more than POT_THRESHOLD
bool readPots(){
    bool newReading = false;
    for (int i = 0; i<NUM_POTS; i++){
    uint16_t newPotReading = analogRead(potPins[i]);
    if (abs(newPotReading - potReadings[i]) > POT_THRESHOLD ){
        potReadings[i] = analogRead(potPins[i]);
        newReading = true;
    }   
    }   
    return newReading;
}

void printPots(){
    Serial.print("Pot Readings: ");
    for (uint16_t potReading: potReadings){
        Serial.printf("%d ", potReading);
    }
    Serial.println();
}