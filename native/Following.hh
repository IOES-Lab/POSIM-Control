// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include "VoyageLimits.hh"

namespace wwos {
struct FollowingCommand { double speed, yawRate; bool braking; };
inline constexpr double FollowingGapMetres = 38.;
// A 38 m separation leaves room for both hulls, wave motion and a stop.
// Separation is measured between the actual hull origins, not waypoints.
inline FollowingCommand Following(double gap, double closing, double leadSpeed,
                                  double error, double bearingError) {
  double speed = std::clamp(leadSpeed + .5 * (gap - FollowingGapMetres),
                            0., FollowingSpeedMetresPerSecond);
  const bool braking = gap < 18. || (closing > 0. &&
      gap < 14. + closing * closing / (2. * 1.5) + closing * .5);
  if (braking) speed = std::max(0., leadSpeed - 4.);
  if (gap < 12.) speed = 0.;
  speed *= std::pow(std::max(0., std::cos(error)), 2.);
  // When holding station, point the forward camera at the leader.
  const double aim = gap < FollowingGapMetres + 4. && leadSpeed < 1. ? bearingError : error;
  return {speed, std::clamp(aim * 1.4, -.5, .5), braking};
}
}
