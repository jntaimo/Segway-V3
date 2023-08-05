#include "util.h"
//map from an input to output range logarithmically
double logMap(double x, double in_min, double in_max, double out_min, double out_max) {
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;

  // Normalize the input value to the [0, 1] range
  double normalized = (log(x) - log(in_min)) / (log(in_max) - log(in_min));

  // Map the normalized value to the output range
  return normalized * (out_max - out_min) + out_min;
}

//map from an input to output range linearly
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;

  // Normalize the input value to the [0, 1] range
  double normalized = (x - in_min) / (in_max - in_min);

  // Map the normalized value to the output range
  return normalized * (out_max - out_min) + out_min;
}
