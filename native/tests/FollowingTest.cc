#include "../Following.hh"
#include "../AttitudeSafety.hh"
#include <cassert>
int main() {
  assert(wwos::UprightThrustScale(1.) == 1.);
  assert(wwos::UprightThrustScale(-1.) == 0.);
  assert(wwos::UprightThrustScale(std::cos(M_PI/2.)) == 0.);
  assert(wwos::UprightThrustScale(std::cos(M_PI/4.)) > 0.);
  assert(wwos::UprightThrustScale(std::cos(M_PI/4.)) < .6);
  // At motorboat speed, limit lateral acceleration rather than applying the
  // same 0.5 rad/s station-keeping turn to a fast moving follower.
  assert(std::abs(wwos::SafeTurnRate(10., 1.) - .12) < 1e-12);
  assert(wwos::SafeTurnRate(0., 1.) == .35);
  assert(wwos::SafeTurnRate(10., -1.) == 0.);
  auto cruise = wwos::Following(38, 0, 10.5, 0, 0);
  assert(cruise.speed == 10.5 && !cruise.braking);
  assert(wwos::Following(53, -1, 10.5, 0, 0).speed == 11.9);
  assert(wwos::Following(28, 0, 10.5, 0, 0).speed == 5.5);
  assert(wwos::Following(38, 0, 14., 0, 0).speed == 11.9);
  auto stop = wwos::Following(20, 8, 0, 0, 0);
  assert(stop.braking && stop.speed == 0);
  assert(wwos::Following(10, 0, 12.5, 0, 0).speed == 0);
  assert(wwos::Following(38, 0, 12.5, 1.6, 0).speed == 0);
  auto aim = wwos::Following(38, 0, 0, 0, .2);
  assert(aim.yawRate > .27 && aim.yawRate < .29);
}
