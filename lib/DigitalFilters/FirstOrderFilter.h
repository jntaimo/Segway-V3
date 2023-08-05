#include <Arduino.h>

class FirstOrderFilter {
public:
  FirstOrderFilter(double a0, double a1, double b0, double b1);
  double calculate(double input); 

private:
  double _a0, _a1, _b0, _b1; // Filter coefficients
  double _previousInput;     // Previous input, used in the difference equation
  double _previousOutput;    // Previous output, used in the difference equation
  unsigned long _lastTime;   // Previous time stamp, used to calculate T
};
