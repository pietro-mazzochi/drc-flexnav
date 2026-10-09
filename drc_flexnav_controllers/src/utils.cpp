#include "drc_flexnav_controllers/utils.hpp"
#include <math.h>

double getLongitudinalVelocity(const double& vx, const double& vy, const double& theta){
  if (sin(theta) < std::numeric_limits<double>::epsilon())
    return vx/cos(theta);

  if (cos(theta) < std::numeric_limits<double>::epsilon())
    return vy/sin(theta);

  return 0.5*(vx/cos(theta) + vy/sin(theta));
}
