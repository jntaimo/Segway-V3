#include <Arduino.h>
#include "pots.h"
#include "pinout.h"

#define NUM_POTS 4
#define POT_THRESHOLD 3
#define NUM_OVERSAMPLE_BITS 6
const int samples = 1 << NUM_OVERSAMPLE_BITS; // Number of samples for oversampling
const uint8_t potPins[] = {POT_1_PIN, POT_2_PIN, POT_3_PIN, POT_4_PIN};

uint16_t potReadings[NUM_POTS];

void potsSetup(){
    analogReadResolution(10);
    // Configure the potentiometers as inputs
    for (uint8_t i: potPins){
        pinMode(i, INPUT);
    }
}

// Function to perform oversampling and averaging
uint16_t oversampledRead(uint8_t pin) {
    long sum = 0;

    for (int i = 0; i < samples; i++) {
        sum += analogRead(pin);
    }

    // Average the readings and increase resolution
    return sum >> NUM_OVERSAMPLE_BITS; // Division by factor of 2
}

// Reads all of the potentiometers and updates the potReadings array
// Returns true if any of the potentiometers have changed by more than POT_THRESHOLD
bool readPots(){
    bool newReading = false;
    for (int i = 0; i < NUM_POTS; i++){
        uint16_t newPotReading = oversampledRead(potPins[i]);
        if (abs(newPotReading - potReadings[i]) > POT_THRESHOLD ){
            potReadings[i] = newPotReading;
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
