#ifndef EncoderVelocity_h
#define EncoderVelocity_h

#include <Arduino.h>
#include <Encoder.h>

class EncoderVelocity {
public:
  // Constructor: Initializes with the given pins and counts per revolution (CPR)
  EncoderVelocity(int pinA, int pinB, int countsPerRevolution);

  // Returns the current velocity in radians per second
  float getVelocity();

  // Returns the current position in radians
  float getPosition();

  // Resets the encoder position readings
  void resetPosition();

private:
  Encoder _encoder;         // Encoder object for reading the encoder
  long _lastPosition;       // Last recorded encoder position (in counts)
  unsigned long _lastTime;  // Time of the last encoder change (in microseconds)
  float _velocity;          // Estimated velocity (in radians per second)
  float _countsToRadians;   // Conversion factor from counts to radians
};

#endif
