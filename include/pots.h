#ifndef POTS_H
#define POTS_H
#include <Arduino.h>
bool readPots();
void printPots();
void potsSetup();
extern uint16_t potReadings[4];
#endif // POTS_H