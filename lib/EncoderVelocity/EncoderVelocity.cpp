#include "EncoderVelocity.h"

// Constructor: Initializes the encoder and conversion factor from counts to radians
EncoderVelocity::EncoderVelocity(int pinA, int pinB, int countsPerRevolution)
  : _encoder(pinA, pinB), _lastPosition(0), _lastTime(0), _velocity(0),
    _countsToRadians(2 * PI / countsPerRevolution) {
  // _countsToRadians converts from encoder counts to radians
}

// Calculates and returns the current velocity in radians per second
float EncoderVelocity::getVelocity() {
  // Read the current encoder position
  long currentPosition = _encoder.read();

  // Get the current time in microseconds
  unsigned long currentTime = micros();

  // Calculate the time difference, handling potential rollover of micros()
  unsigned long dt = (currentTime >= _lastTime) ? (currentTime - _lastTime) : (currentTime + (0xFFFFFFFF - _lastTime));

  // Check if the encoder position has changed and dt is not zero
  if (currentPosition != _lastPosition && dt > 0) {
    // Calculate the velocity as the change in position (in radians) divided by the change in time (in seconds)
    _velocity = (float)(currentPosition - _lastPosition) * _countsToRadians / dt * 1000000;

    // Update the last position and time
    _lastPosition = currentPosition;
    _lastTime = currentTime;
  }

  // Return the velocity in radians per second
  return _velocity;
}

// Returns the current position of the encoder in radians
float EncoderVelocity::getPosition() {
  // Read the current encoder position (in counts) and convert it to radians
  return _encoder.read() * _countsToRadians;
}

// Resets the position readings of the encoder
void EncoderVelocity::resetPosition() {
  // Reset the encoder's position reading to zero (in counts)
  _encoder.write(0);

  // Reset internal tracking of the last position (in counts) and time (in microseconds)
  _lastPosition = 0;
  _lastTime = micros();
}
