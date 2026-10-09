// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>

namespace wwos {
// Reduce commanded propulsion on a badly tilted hull. This does not add a
// restoring torque, move the hull or alter the upstream mass/hydrodynamics.
inline double UprightThrustScale(double upZ) {
  const double tilt = std::acos(std::clamp(upZ, -1., 1.));
  return std::clamp((70.*M_PI/180.-tilt)/(45.*M_PI/180.), 0., 1.);
}
inline double SafeTurnRate(double forwardSpeed, double upZ) {
  return std::min(.35, 1.2/std::max(1., std::abs(forwardSpeed))) * UprightThrustScale(upZ);
}
}
