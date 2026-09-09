#ifndef MPCC_DIAGNOSTIC_ACTUATOR_CONTACT_CONTRACT_HPP
#define MPCC_DIAGNOSTIC_ACTUATOR_CONTACT_CONTRACT_HPP

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <utility>

namespace mpcc_actuator_contact_contract
{
// Diagnostic CIL transcription at the actual receiver/physics boundary.
// It cannot infer DDS receipt, application phase or future contact from a
// controller publication stamp. No authority or substitute clock is supplied.
struct SteeringParameters
{
  float delay_sec{.1F};
  float lag_sec{.02F};
  float slew_deg_per_sec{80.F};
  float grip{.7F};
  float input_limit_deg{30.F};
};
struct SteeringState
{
  float tire_deg{};
  std::optional<float> last_fixed_sec;
  std::deque<std::pair<float, float>> demand_history;
};
inline bool valid(const SteeringParameters & p)
{
  return std::isfinite(p.delay_sec) && p.delay_sec >= 0 &&
    std::isfinite(p.lag_sec) && p.lag_sec >= 0 &&
    std::isfinite(p.slew_deg_per_sec) && p.slew_deg_per_sec > 0 &&
    std::isfinite(p.grip) && p.grip > 0 && p.grip <= 1 &&
    std::isfinite(p.input_limit_deg) && p.input_limit_deg > 0;
}
inline std::optional<SteeringState> steering_step(
  SteeringState state, const SteeringParameters & p,
  float fixed_sec, float dt_sec, float receiver_input_deg)
{
  if (!valid(p) || !std::isfinite(fixed_sec) || fixed_sec < 0 ||
    !std::isfinite(dt_sec) || dt_sec <= 0 || !std::isfinite(receiver_input_deg) ||
    !std::isfinite(state.tire_deg) ||
    (state.last_fixed_sec && (!std::isfinite(*state.last_fixed_sec) ||
    *state.last_fixed_sec < 0 || fixed_sec <= *state.last_fixed_sec)))
  {
    return std::nullopt;
  }
  std::optional<float> preceding;
  for (const auto & [stamp, value] : state.demand_history) {
    if (!state.last_fixed_sec || !std::isfinite(stamp) || stamp < 0 ||
      stamp > *state.last_fixed_sec || !std::isfinite(value) ||
      std::abs(value) > p.input_limit_deg * p.grip ||
      (preceding && stamp <= *preceding))
    {
      return std::nullopt;
    }
    preceding = stamp;
  }
  // Limits here reproduce the plant. They are not post-solve command repair.
  const float demand = std::clamp(receiver_input_deg, -p.input_limit_deg, p.input_limit_deg) * p.grip;
  state.demand_history.emplace_back(fixed_sec, demand);
  while (!state.demand_history.empty() &&
    fixed_sec - state.demand_history.front().first > p.delay_sec + .1F)
  {
    state.demand_history.pop_front();
  }
  float delayed = state.tire_deg;
  const float cutoff = fixed_sec - p.delay_sec;
  for (const auto & [stamp, value] : state.demand_history) {
    if (stamp > cutoff) {break;}
    delayed = value;
  }
  const float alpha = dt_sec / (p.lag_sec + dt_sec);
  const float target = state.tire_deg + alpha * (delayed - state.tire_deg);
  const float limit = p.slew_deg_per_sec * dt_sec;
  state.tire_deg += std::clamp(target - state.tire_deg, -limit, limit);
  state.last_fixed_sec = fixed_sec;
  return state;
}

struct DriveParameters
{
  float maximum_acceleration_mps2{1.37F};
  float maximum_deceleration_mps2{8.F};
  float rolling_mps2{.37F};
  float sleep_speed_mps{.02F};
};
struct DriveStep
{
  float limited_wire_mps2{};
  float acceleration_per_grounded_wheel_mps2{};
  bool contact_qualified_sleep{};
};
inline std::optional<DriveStep> drive_step(
  const DriveParameters & p, float speed_mps, float wire_mps2,
  float dt_sec, unsigned grounded_mask)
{
  // Drive-only contract; Reverse/Park and unobserved gear are not silently
  // coerced into Drive. The caller must establish the gear outside this API.
  if (!std::isfinite(speed_mps) || !std::isfinite(wire_mps2) ||
    !std::isfinite(dt_sec) || dt_sec <= 0 || grounded_mask > 15 ||
    !std::isfinite(p.maximum_acceleration_mps2) || p.maximum_acceleration_mps2 <= 0 ||
    !std::isfinite(p.maximum_deceleration_mps2) || p.maximum_deceleration_mps2 <= 0 ||
    !std::isfinite(p.rolling_mps2) || p.rolling_mps2 < 0 ||
    !std::isfinite(p.sleep_speed_mps) || p.sleep_speed_mps <= 0)
  {
    return std::nullopt;
  }
  DriveStep result;
  result.limited_wire_mps2 = std::clamp(
    wire_mps2, -p.maximum_deceleration_mps2, p.maximum_acceleration_mps2);
  result.contact_qualified_sleep = grounded_mask == 15 &&
    std::abs(speed_mps) < p.sleep_speed_mps && result.limited_wire_mps2 <= 0;
  if (result.contact_qualified_sleep) {return result;}
  float acceleration = result.limited_wire_mps2;
  const float time = std::max(dt_sec, .01F);
  if (speed_mps < 0) {acceleration = std::max(acceleration, -speed_mps / time);}
  acceleration = std::clamp(
    acceleration, -p.maximum_deceleration_mps2, p.maximum_acceleration_mps2);
  const float rolling = speed_mps == 0 ? 0 :
    -std::copysign(std::min(p.rolling_mps2, std::abs(speed_mps) / time), speed_mps);
  result.acceleration_per_grounded_wheel_mps2 = (acceleration + rolling) / 4;
  return result;
}
}  // namespace mpcc_actuator_contact_contract
#endif
