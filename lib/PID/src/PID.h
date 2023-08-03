#ifndef PID_H
#define PID_H

class PID {
public:
  PID(double Kp, double Ki, double Kd, double setpoint, double tau, bool serial);

  // Set tuning parameters for parallel PID form
  void setParallelTunings(double Kp, double Ki, double Kd);
  void setParallelTunings(double Kp, double Ki, double Kd, double tau, double integralMin, double integralMax);
  // Set tuning parameters for serial PID form
  void setSerialTunings(double Kp, double Ti, double Td);
  void setSerialTunings(double Kp, double Ti, double Td, double tau, double integralMin, double integralMax);

  // Parallel form PID calculation
  double calculateParallel(double input, double setpoint);

  // Serial form PID calculation
  double calculateSerial(double input, double setpoint);

private:
  unsigned long handleMicrosRollover(unsigned long currentTime);

  double _Kp, _Ki, _Kd; // Proportional, Integral, and Derivative gains/time constants
  double _previousError; // Previous error, used for derivative calculation
  double _integral; // Integral term accumulation
  double _setpoint; // Desired setpoint
  double _lastDerivative; // Last derivative calculation, used for filtering
  double _tau; // Tau value for the derivative low-pass filter
  double _alpha; // Alpha value for the derivative low-pass filter
  double _integralMin, _integralMax; // Limits for integral windup prevention
  unsigned long _previousTime; // Previous time in microseconds, used to calculate dt
};

#endif // PID_H
