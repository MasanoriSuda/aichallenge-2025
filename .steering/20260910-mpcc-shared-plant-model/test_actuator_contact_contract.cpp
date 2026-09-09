#include "actuator_contact_contract.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace plant = mpcc_actuator_contact_contract;
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}
int main(int argc, char ** argv)
{
  std::cout << std::setprecision(17);
  if (argc == 2 && std::string(argv[1]) == "replay") {
    plant::SteeringState steering;
    bool initialized = false;
    double time{}, input{}, actual_tire{}, speed{}, wire{}, dt{};
    unsigned contact{};
    while (std::cin >> time >> dt >> input >> actual_tire >> speed >> wire >> contact) {
      if (!initialized) {steering.tire_deg = static_cast<float>(actual_tire); initialized = true;}
      const auto next = plant::steering_step(steering, {}, static_cast<float>(time),
        static_cast<float>(dt), static_cast<float>(input));
      const auto drive = plant::drive_step({}, static_cast<float>(speed),
        static_cast<float>(wire), static_cast<float>(dt), contact);
      require(next && drive, "invalid recorded actuator request");
      steering = *next;
      std::cout << steering.tire_deg << ' ' << drive->acceleration_per_grounded_wheel_mps2 <<
        ' ' << drive->contact_qualified_sleep << '\n';
    }
    require(std::cin.eof(), "malformed replay stream");
    return 0;
  }
  plant::SteeringState state;
  for (int step = 0; step < 20; ++step) {
    auto next = plant::steering_step(state, {}, step * .005F, .005F, 30.F);
    require(next && next->tire_deg == 0, "future demand crossed the delay boundary");
    state = *next;
  }
  auto changed = plant::steering_step(state, {}, .1F, .005F, 30.F);
  require(changed && std::abs(changed->tire_deg - .4F) < 1e-6F, "slew-limited delay response");
  require(!plant::steering_step(*changed, {}, .1F, .005F, 0), "duplicate physics epoch accepted");
  require(!plant::steering_step(*changed, {}, .09F, .005F, 0), "backward physics epoch accepted");
  require(!plant::steering_step(state, {}, .1F, .005F,
    std::numeric_limits<float>::quiet_NaN()), "invalid input accepted");
  auto future = state;
  future.demand_history.emplace_back(.2F, 20.F);
  require(!plant::steering_step(future, {}, .1F, .005F, 0), "future queued demand accepted");
  auto invalid_clock = state;
  invalid_clock.last_fixed_sec = std::numeric_limits<float>::quiet_NaN();
  require(!plant::steering_step(invalid_clock, {}, .1F, .005F, 0), "nonfinite source clock accepted");
  for (unsigned mask = 0; mask < 16; ++mask) {
    const auto unforced = plant::drive_step({}, 0, 0, .005F, mask);
    const auto brake = plant::drive_step({}, 0, -3, .005F, mask);
    const auto launch = plant::drive_step({}, 0, 1, .005F, mask);
    require(unforced && unforced->acceleration_per_grounded_wheel_mps2 == 0,
      "unforced rest creates acceleration");
    require(brake && brake->contact_qualified_sleep == (mask == 15),
      "sleep assumed without all-wheel contact");
    require(brake->acceleration_per_grounded_wheel_mps2 == (mask == 15 ? 0.F : -.75F),
      "brake at rest does not follow conditional CIL branch");
    require(launch && !launch->contact_qualified_sleep &&
      launch->acceleration_per_grounded_wheel_mps2 == .25F, "launch held by stale sleep");
  }
  const auto reverse_drift = plant::drive_step({}, -.001F, -3, .005F, 7);
  require(reverse_drift && std::abs(reverse_drift->acceleration_per_grounded_wheel_mps2 - .05F) < 1e-7F,
    "Drive restorative response omitted");
  require(!plant::drive_step({}, 0, 0, 0, 15), "zero time accepted");
  require(!plant::drive_step({}, 0, 0, .005F, 16), "invalid contact mask accepted");
  std::cout << "Passed delay boundary, slew, source monotonicity,16contact/rest/brake/launch modes and invalid input checks.\n";
}
