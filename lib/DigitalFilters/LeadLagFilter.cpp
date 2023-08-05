#include "LeadLagFilter.h"
#include <Arduino.h>

LeadLagFilter::LeadLagFilter(double alpha, double tau)
  : _alpha(alpha), _tau(tau), _filter(_tau, 1, _alpha*_tau, 1) {
}

double LeadLagFilter::calculate(double input) {
  // Calculate filter coefficients

  // Apply the filter
  double output = _filter.calculate(input);

  return output;
}
