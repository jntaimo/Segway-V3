#include "FirstOrderFilter.h"

FirstOrderFilter::FirstOrderFilter(double a0, double a1, double b0, double b1)
  : _a0(a0), _a1(a1), _b0(b0), _b1(b1), _previousInput(0), _previousOutput(0) , _lastTime(micros()){
}

double FirstOrderFilter::calculate(double input) {
    //based on https://en.wikipedia.org/wiki/Bilinear_transform#Transformation_for_a_general_first-order_continuous-time_filter
    unsigned long currentTime = micros();
    //calculate sample time T
    unsigned long T = (currentTime >= _lastTime) ? (currentTime - _lastTime) : (currentTime + (0xFFFFFFFF - _lastTime));
    double K = 2.0/T;
    double output = (_b0 * K + _b1)/(_a0 * K + _a1) * input + (-_b0 * K + _b1)/(_a0 * K + _a1) * _previousInput - (-_a0 * K + _a1)/(_a0 * K + _a1) * _previousOutput;
    _previousInput = input;
    _previousOutput = output;
    _lastTime = currentTime;
    return output;
}

