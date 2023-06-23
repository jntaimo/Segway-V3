#include <Arduino.h>
#include "pots.h"

#define POT_1_PIN 1
#define POT_2_PIN 2
#define POT_3_PIN 4
#define POT_4_PIN 5

#define NUM_POTS 4
const uint8_t potPins[] = {POT_1_PIN, POT_2_PIN, POT_3_PIN, POT_4_PIN};
uint16_t potReadings[4];

void potsSetup(){
    Serial.begin(115200);
    //configure the potentiometers as inputs
    for (uint8_t i: potPins){
        pinMode(i, INPUT);
    }
}


void readPots(){
    for (int i = 0; i<NUM_POTS; i++){
    potReadings[i] = analogRead(potPins[i]);
    }   
}

void printPots(){
    Serial.print("Pot Readings: ");
    for (uint16_t potReading: potReadings){
        Serial.printf("%d ", potReading);
    }
    Serial.println();
}