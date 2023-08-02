#ifndef PID_H
#define PID_H

class PID {
public:
  PID(double Kp, double Ki, double Kd, double setpoint);

  // Set tuning parameters for parallel PID form
  void setParallelTunings(double Kp, double Ki, double Kd);

  // Set tuning parameters for serial PID form
  void setSerialTunings(double Kp, double Ti, double Td);

  // Parallel form PID calculation
  double calculateParallel(double input);

  // Serial form PID calculation
  double calculateSerial(double input);

private:
  unsigned long handleMicrosRollover(unsigned long currentTime);

  double _Kp, _Ki, _Kd; // Proportional, Integral, and Derivative gains/time constants
  double _previousError; // Previous error, used for derivative calculation
  double _integral; // Integral term accumulation
  double _setpoint; // Desired setpoint
  double _lastDerivative; // Last derivative calculation, used for filtering
  double _alpha; // Alpha value for the derivative low-pass filter
  double _integralMin, _integralMax; // Limits for integral windup prevention
  unsigned long _previousTime; // Previous time in microseconds, used to calculate dt
};

#endif // PID_H
