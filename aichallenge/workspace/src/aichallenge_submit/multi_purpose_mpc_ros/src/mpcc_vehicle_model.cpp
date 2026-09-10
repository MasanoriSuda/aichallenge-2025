#include "multi_purpose_mpc_ros/mpcc_vehicle_model.hpp"
#include "multi_purpose_mpc_ros/mpcc_vehicle_model_kernel.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace multi_purpose_mpc_ros::mpcc_vehicle_model
{
namespace
{
constexpr double kHalfPi = 1.57079632679489661923;

bool positive(const double value) noexcept
{
  return std::isfinite(value) && value > 0;
}

bool nonnegative(const double value) noexcept
{
  return std::isfinite(value) && value >= 0;
}

State body_increment(State state, const BodyDerivative & rate, const double dt) noexcept
{
  return kernel::state(kernel::body_increment(kernel::values(state),
    kernel::BodyRates<double>{rate.x_mps, rate.y_mps, rate.yaw_radps,
      rate.forward_acceleration_mps2, rate.lateral_acceleration_mps2,
      rate.yaw_acceleration_radps2}, dt));
}
}  // namespace

std::size_t integration_steps(const double duration, const double maximum_step) noexcept
{
  if (!nonnegative(duration) || !positive(maximum_step)) return 0;
  const double ratio = duration / maximum_step;
  if (!std::isfinite(ratio) || ratio > 100000) return 0;
  // Floating subtraction of absolute timestamps can put an exact nominal
  // interval a few ULPs above an integer. Do not invent an extra mechanical
  // tire update at that boundary. This changes no physical constraint margin.
  const double ulps = 16 * std::numeric_limits<double>::epsilon() * std::max(1.0, ratio);
  return duration == 0.0 ? 0 : static_cast<std::size_t>(std::max(1.0, std::ceil(ratio - ulps)));
}

bool valid(const Parameters & p) noexcept
{
  if (p.profile_id.empty() || !positive(p.mass_kg) || !positive(p.yaw_inertia_kgm2) ||
    !std::isfinite(p.com_forward_m) || !std::isfinite(p.com_left_m) ||
    !nonnegative(p.rolling_mps2) || !nonnegative(p.drag_per_sec) ||
    !nonnegative(p.angular_drag_per_sec) || !positive(p.maximum_wire_acceleration_mps2) ||
    !positive(p.maximum_wire_deceleration_mps2) || !positive(p.sleep_speed_mps) ||
    !positive(p.minimum_force_interval_sec) || !nonnegative(p.tire_lag_sec) ||
    !positive(p.tire_slew_radps) || !positive(p.tire_grip) || p.tire_grip > 1 ||
    !positive(p.steering_wire_gain) || !positive(p.maximum_wire_steering_rad) ||
    p.maximum_wire_steering_rad >= kHalfPi || !positive(p.maximum_step_sec) ||
    p.maximum_step_sec > p.minimum_force_interval_sec)
  {
    return false;
  }
  for (const auto & wheel : p.wheels) {
    if (!std::isfinite(wheel.com_forward_m) || !std::isfinite(wheel.com_left_m) ||
      !nonnegative(wheel.traction_fraction) || wheel.traction_fraction > .25 ||
      !nonnegative(wheel.cornering_per_sec))
    {
      return false;
    }
  }
  return true;
}

bool finite(const State & s) noexcept
{
  return std::isfinite(s.x_m) && std::isfinite(s.y_m) && std::isfinite(s.yaw_rad) &&
         std::isfinite(s.forward_velocity_mps) && std::isfinite(s.lateral_velocity_mps) &&
         std::isfinite(s.yaw_rate_radps) && std::isfinite(s.desired_steering_rad) &&
         std::isfinite(s.tire_steering_rad) && std::abs(s.tire_steering_rad) < kHalfPi;
}

std::uint64_t fingerprint(const Parameters & p) noexcept
{
  if (!valid(p)) {return 0;}
  std::uint64_t hash = 14695981039346656037ULL;
  const auto byte = [&hash](const unsigned char value) {
      hash = (hash ^ value) * 1099511628211ULL;
    };
  const auto integer = [&byte](const std::uint64_t value) {
      for (unsigned shift = 0; shift < 64; shift += 8) {
        byte(static_cast<unsigned char>(value >> shift));
      }
    };
  const auto number = [&integer](const double value) {
      std::uint64_t bits{};
      static_assert(sizeof(bits) == sizeof(value));
      std::memcpy(&bits, &value, sizeof(bits));
      integer(bits);
    };
  integer(1);  // Parameter encoding version, distinct from the state schema.
  integer(p.profile_id.size());
  for (const unsigned char value : p.profile_id) {byte(value);}
  for (const double value : {p.mass_kg, p.yaw_inertia_kgm2, p.com_forward_m, p.com_left_m,
    p.rolling_mps2, p.drag_per_sec, p.angular_drag_per_sec, p.maximum_wire_acceleration_mps2,
    p.maximum_wire_deceleration_mps2, p.sleep_speed_mps, p.minimum_force_interval_sec,
    p.tire_lag_sec, p.tire_slew_radps, p.tire_grip, p.steering_wire_gain,
    p.maximum_wire_steering_rad, p.maximum_step_sec})
  {
    number(value);
  }
  integer(p.nominal_settled_contact);
  for (const auto & wheel : p.wheels) {
    number(wheel.com_forward_m); number(wheel.com_left_m);
    number(wheel.traction_fraction); number(wheel.cornering_per_sec);
    integer(wheel.steerable);
  }
  return hash == 0 ? 1 : hash;
}

std::optional<BodyDerivative> derivative(
  const State & s, const double wire, const Parameters & p,
  const double force_interval_sec) noexcept
{
  if (!valid(p) || !finite(s) || !std::isfinite(wire) || !positive(force_interval_sec)) {
    return std::nullopt;
  }
  const auto rates = kernel::derivative(
    kernel::values(s), wire, p, force_interval_sec, kernel::ScalarArithmetic{});
  const BodyDerivative result{rates[0], rates[1], rates[2], rates[3], rates[4], rates[5]};
  for (const double value : {result.x_mps, result.y_mps, result.yaw_radps,
    result.forward_acceleration_mps2, result.lateral_acceleration_mps2,
    result.yaw_acceleration_radps2})
  {
    if (!std::isfinite(value)) {return std::nullopt;}
  }
  return result;
}

std::optional<Transition> advance(
  State state, const Input & input, const Parameters & p,
  const double duration_sec) noexcept
{
  if (!valid(p) || !finite(state) || !std::isfinite(input.wire_acceleration_mps2) ||
    !std::isfinite(input.desired_steering_rate_radps) || !nonnegative(duration_sec) ||
    duration_sec / p.maximum_step_sec > 100000)
  {
    return std::nullopt;
  }
  if (duration_sec == 0) {return Transition{state, 0, false};}
  const auto count = integration_steps(duration_sec, p.maximum_step_sec);
  const double dt = duration_sec / static_cast<double>(count);
  bool nominal_rest_applied = false;
  for (std::size_t step = 0; step < count; ++step) {
    const double wire_steering = state.desired_steering_rad * p.steering_wire_gain;
    if (!std::isfinite(wire_steering)) {return std::nullopt;}
    // These bounds transcribe the plant response; the command and its original
    // physical bounds remain unchanged and separately certified by the caller.
    auto values = kernel::values(state);
    kernel::update_tire(values, p, dt, kernel::ScalarArithmetic{});
    state = kernel::state(values);
    const bool rest = p.nominal_settled_contact &&
      std::abs(state.forward_velocity_mps) < p.sleep_speed_mps &&
      input.wire_acceleration_mps2 <= 0;
    if (rest) {
      state.forward_velocity_mps = 0;
      state.lateral_velocity_mps = 0;
      state.yaw_rate_radps = 0;
      nominal_rest_applied = true;
    } else {
      const auto first = derivative(state, input.wire_acceleration_mps2, p, dt);
      if (!first) {return std::nullopt;}
      const auto middle = body_increment(state, *first, .5 * dt);
      const auto second = derivative(middle, input.wire_acceleration_mps2, p, dt);
      if (!second) {return std::nullopt;}
      state = body_increment(state, *second, dt);
    }
    state.desired_steering_rad += input.desired_steering_rate_radps * dt;
    if (!finite(state)) {return std::nullopt;}
  }
  return Transition{state, count, nominal_rest_applied};
}

std::optional<double> nominal_stop_map_distance(
  const Parameters & parameters, const double speed_mps,
  const double maximum_wire_acceleration_mps2,
  const double braking_wire_acceleration_mps2,
  const double publication_interval_sec) noexcept
{
  if (!valid(parameters) || !std::isfinite(speed_mps) || speed_mps < 0.0 ||
    !std::isfinite(maximum_wire_acceleration_mps2) ||
    !std::isfinite(braking_wire_acceleration_mps2) ||
    braking_wire_acceleration_mps2 >= 0.0 ||
    !std::isfinite(publication_interval_sec) || publication_interval_sec <= 0.0)
  {
    return std::nullopt;
  }
  auto prefix = advance({0, 0, 0, speed_mps, 0, 0, 0, 0},
    {maximum_wire_acceleration_mps2, 0}, parameters, publication_interval_sec);
  if (!prefix) return std::nullopt;
  State state = prefix->state;
  for (std::size_t step = 0; step < 10000U; ++step) {
    if (state.forward_velocity_mps == 0.0 && state.lateral_velocity_mps == 0.0 &&
      state.yaw_rate_radps == 0.0) return state.x_m;
    const auto next = advance(state, {braking_wire_acceleration_mps2, 0},
      parameters, parameters.maximum_step_sec);
    if (!next || next->state.forward_velocity_mps < 0.0) return std::nullopt;
    state = next->state;
  }
  return std::nullopt;
}

}  // namespace multi_purpose_mpc_ros::mpcc_vehicle_model
