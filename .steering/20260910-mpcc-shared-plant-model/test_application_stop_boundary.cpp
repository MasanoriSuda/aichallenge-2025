#include "actuator_contact_contract.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace plant = mpcc_actuator_contact_contract;
// Isolated forward/straight planar component, with all-wheel ground supplied
// as an assumption. This is neither a complete rigidbody nor a road model.
struct State {double x{}, speed{1.5}; int sleep_tick{-1};};

State step(State s, const double wire, const unsigned contact, const int tick)
{
  constexpr double dt = .005;
  const auto drive = plant::drive_step({}, static_cast<float>(s.speed),
    static_cast<float>(wire), static_cast<float>(dt), contact);
  if (!drive) {throw std::runtime_error("invalid plant request");}
  if (drive->contact_qualified_sleep) {
    s.speed = 0;
    if (s.sleep_tick < 0) {s.sleep_tick = tick;}
  } else {
    unsigned count{};
    for (unsigned n = 0; n < 4; ++n) {count += (contact >> n) & 1U;}
    const double acceleration = count * drive->acceleration_per_grounded_wheel_mps2;
    s.speed = (s.speed+acceleration*dt)*(1-.03*dt);
    s.x += s.speed*dt;
  }
  return s;
}

// The exact same receive stream publishes brake for one 25ms publisher
// interval, then acceleration. A 10Hz latest-value selector can either select
// or overwrite the brake, depending on its phase. No DDS loss is needed.
State pulse(const int application_phase_ticks)
{
  State s;
  double applied = 1.37;
  for (int k = 0; k < 40; ++k) {
    const double received = k < 5 ? -3 : 1.37;
    if (k >= application_phase_ticks && (k-application_phase_ticks)%20 == 0) {
      applied = received;
    }
    s = step(s, applied, 15, k);
  }
  return s;
}

State stop(const int brake_receive_tick, const unsigned contact)
{
  State s;
  double applied = 1.37;
  for (int k = 0; k < 400; ++k) {
    if (k%20 == 0) {applied = k >= brake_receive_tick ? -3 : 1.37;}
    s = step(s, applied, contact, k);
  }
  return s;
}

void require(const bool value, const char * message)
{
  if (!value) {throw std::runtime_error(message);}
}

int main()
{
  const auto selected = pulse(0), overwritten = pulse(6);
  require(overwritten.speed-selected.speed > .3,
    "same publication stream unexpectedly defines one applied trajectory");
  const auto immediate = stop(0, 15), delayed = stop(100, 15);
  const auto missing_receive = stop(400, 15), missing_contact = stop(0, 0);
  require(immediate.sleep_tick >= 0 && delayed.sleep_tick > immediate.sleep_tick,
    "declared complete-contact braking scenario does not reach conditional sleep");
  require(delayed.x > immediate.x+.5, "receive uncertainty does not affect Stop support");
  require(missing_receive.sleep_tick < 0 && missing_receive.speed > 1.5,
    "nonreceived brake was incorrectly executed");
  require(missing_contact.sleep_tick < 0 && missing_contact.speed > 1,
    "no-contact planar abstraction was silently assigned braking force");
  // No-contact for two seconds is an unclosed abstraction branch. This test
  // does not establish that such a contact history is reachable on this road.
  // Constraining it needs road/vertical physics, not changing a proof tolerance.
  std::cout << std::setprecision(17)
    << "{\"authority\":false,\"publisher_pulse_sec\":0.025,"
    << "\"application_period_sec\":0.1,\"phase_sec\":[0,0.03],"
    << "\"selected_pulse\":{\"x_m\":" << selected.x << ",\"v_mps\":" << selected.speed << "},"
    << "\"overwritten_pulse\":{\"x_m\":" << overwritten.x << ",\"v_mps\":" << overwritten.speed << "},"
    << "\"immediate_stop\":{\"x_m\":" << immediate.x << ",\"sleep_tick\":" << immediate.sleep_tick << "},"
    << "\"delayed_stop\":{\"x_m\":" << delayed.x << ",\"sleep_tick\":" << delayed.sleep_tick << "},"
    << "\"brake_unreceived_speed_mps\":" << missing_receive.speed << ','
    << "\"unconstrained_no_contact_speed_mps\":" << missing_contact.speed << "}\n";
}
