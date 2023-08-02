#include <Arduino.h>

class PID {
public:
  PID(double Kp, double Ki, double Kd, double setpoint)
    : _Kp(Kp), _Ki(Ki), _Kd(Kd), _setpoint(setpoint), _previousError(0), _integral(0), _lastDerivative(0), _alpha(1), _integralMin(0), _integralMax(0) {
    _previousTime = micros(); // Store the current time in microseconds
  }

  // Set tuning parameters for parallel PID form
  void setParallelTunings(double Kp, double Ki, double Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
  }

  // Set tuning parameters for serial PID form
  void setSerialTunings(double Kp, double Ti, double Td) {
    _Kp = Kp;
    _Ki = 1.0 / Ti; // Integral time constant
    _Kd = Td;       // Derivative time constant
  }

  // Parallel form PID calculation
  double calculateParallel(double input) {
    unsigned long currentTime = micros();
    unsigned long timeDifference = handleMicrosRollover(currentTime);
    double dt = (double)timeDifference / 1000000.0; // Time difference in seconds

    double error = _setpoint - input;               // Calculate error
    _integral += error * dt;                        // Integral term
    _integral = constrain(_integral, _integralMin, _integralMax); // Prevent integral windup

    double derivative = (error - _previousError) / dt; // Derivative term
    derivative = _alpha * _lastDerivative + (1 - _alpha) * derivative; // Apply low-pass filter
    _lastDerivative = derivative;

    double output = _Kp * error + _Ki * _integral + _Kd * derivative; // Calculate parallel PID output
    _previousError = error;

    return output;
  }

  // Serial form PID calculation
  double calculateSerial(double input) {
    unsigned long currentTime = micros();
    unsigned long timeDifference = handleMicrosRollover(currentTime);
    double dt = (double)timeDifference / 1000000.0; // Time difference in seconds

    double error = _setpoint - input;               // Calculate error
    _integral += (1.0 / _Ki) * error * dt;          // Integral term
    _integral = constrain(_integral, _integralMin, _integralMax); // Prevent integral windup

    double derivative = (error - _previousError) / dt; // Derivative term
    derivative = _alpha * _lastDerivative + (1 - _alpha) * derivative; // Apply low-pass filter
    _lastDerivative = derivative;

    double output = _Kp * (error + _integral + _Kd * derivative); // Calculate serial PID output
    _previousError = error;

    return output;
  }

private:
  // Method to handle micros() rollover
  unsigned long handleMicrosRollover(unsigned long currentTime) {
    unsigned long timeDifference;
    if (currentTime < _previousTime) { // Rollover has occurred
      timeDifference = (ULONG_MAX - _previousTime) + currentTime; // Correct time difference
    } else {
      timeDifference = currentTime - _previousTime;
    }
    _previousTime = currentTime;
    return timeDifference;
  }

  double _Kp, _Ki, _Kd; // Proportional, Integral, and Derivative gains/time constants
  double _previousError; // Previous error, used for derivative calculation
  double _integral; // Integral term accumulation
  double _setpoint; // Desired setpoint
  double _lastDerivative; // Last derivative calculation, used for filtering
  double _alpha; // Alpha value for the derivative low-pass filter
  double _integralMin, _integralMax; // Limits for integral windup prevention
  unsigned long _previousTime; // Previous time in microseconds, used to calculate dt
};
