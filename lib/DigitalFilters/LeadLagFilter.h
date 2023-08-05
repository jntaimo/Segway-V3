
#include "FirstOrderFilter.h"

class LeadLagFilter {
public:
  LeadLagFilter(double alpha, double tau);
  double calculate(double input);

private:
  double _alpha; // Alpha parameter of the lead filter
  double _tau;   // Tau parameter of the lead filter
  FirstOrderFilter _filter; // Instance of the general first-order filter
};
