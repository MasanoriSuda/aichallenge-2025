#include "planar_body_model.hpp"

#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace model = mpcc_planar_body_model;
void require(bool result, const char * message)
{
  if (!result) {throw std::runtime_error(message);}
}
int main()
{
  model::Parameters p;
  p.mass_kg = 160;
  p.yaw_inertia_kgm2 = 82.59312438964844;
  p.com_forward_m = -0.3090076605226058;
  for (std::size_t i = 0; i < 4; ++i) {
    p.wheels[i] = {i < 2 ? .912 : -.175, i % 2 == 0 ? .5 : -.5, .25, 0, i < 2};
  }
  std::cout << std::setprecision(17);
  auto rest = model::transition({}, {0, .3}, p, .2);
  require(rest && rest->x == 0 && rest->y == 0 && rest->forward_velocity == 0 &&
    rest->lateral_velocity == 0 && rest->yaw_rate == 0, "unforced rest moves");

  p.rolling_mps2 = .37;
  const auto accelerating = model::transition({0, 0, 0, 2, 0, 0}, {1, 0}, p, .2);
  const auto braking = model::transition({0, 0, 0, 2, 0, 0}, {-1, 0}, p, .2);
  require(accelerating && std::abs(accelerating->forward_velocity - 2.126) < 1e-12,
    "rolling-only acceleration mismatch");
  require(braking && std::abs(braking->forward_velocity - 1.726) < 1e-12,
    "rolling-only braking mismatch");
  std::cout << "rolling positive=" << accelerating->forward_velocity <<
    " negative=" << braking->forward_velocity << '\n';

  p.rolling_mps2 = 0;
  for (auto & wheel : p.wheels) {wheel.traction_fraction = 0;}
  // During no-contact motion, the COM follows an inertial straight line even
  // while the body rotates. The reference origin must rotate about that COM.
  auto free = model::transition({0, 0, .4, 2, .1, 1}, {1, .3}, p, .2);
  const double x_expected = (2 * std::cos(.4) - .1 * std::sin(.4)) * .2 +
    p.com_forward_m * (std::cos(.4) - std::cos(.6));
  const double y_expected = (2 * std::sin(.4) + .1 * std::cos(.4)) * .2 +
    p.com_forward_m * (std::sin(.4) - std::sin(.6));
  require(free && std::abs(free->x - x_expected) < 1e-10 &&
    std::abs(free->y - y_expected) < 1e-10, "reference/COM transport mismatch");
  std::cout << "free rotation reference error=" << std::hypot(free->x-x_expected, free->y-y_expected) << '\n';

  for (auto & wheel : p.wheels) {wheel.cornering_per_sec = 10;}
  for (double u : {0., 1., 8.}) {
    for (double vy : {-.7, 0., .7}) {
      for (double r : {-1., 0., 1.}) {
        for (double delta : {-.3, 0., .3}) {
          const auto d = model::derivative({0,0,.4,u,vy,r}, {0,delta}, p);
          require(d.has_value(), "valid derivative rejected");
          const double power = p.mass_kg * (u*d->forward_velocity + vy*d->lateral_velocity) +
            p.yaw_inertia_kgm2 * r*d->yaw_rate;
          require(power <= 1e-9, "unforced tire forces create kinetic energy");
        }
      }
    }
  }
  auto invalid = p;
  invalid.wheels[0].cornering_per_sec = -1;
  require(!model::transition({}, {}, invalid, .2), "negative tire damping accepted");
  require(!model::transition({}, {}, p, -1), "negative time accepted");
  require(!model::transition({}, {std::numeric_limits<double>::quiet_NaN(), 0}, p, .2),
    "nonfinite wire input accepted");
  std::cout << "Passed rest, rolling, COM/reference transport,81passivity cases and invalid inputs.\n";
  std::cout << "Moving-mode diagnostic only: discrete sleep/gear and command application contract remain open.\n";
}
