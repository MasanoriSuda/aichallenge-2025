#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_VEHICLE_MODEL_KERNEL_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_VEHICLE_MODEL_KERNEL_HPP_

#include "multi_purpose_mpc_ros/mpcc_vehicle_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace multi_purpose_mpc_ros::mpcc_vehicle_model::kernel
{

// Internal arithmetic shared by scalar rollout and numerical enclosures.
// The caller owns validation, hybrid mode selection and input application time.
enum Coordinate : std::size_t {X, Y, Yaw, Forward, Left, YawRate, Desired, Tire};
template<class Scalar> using StateValues = std::array<Scalar, 8>;
template<class Scalar> using BodyRates = std::array<Scalar, 6>;

inline StateValues<double> values(const State & s) noexcept
{
  return {s.x_m, s.y_m, s.yaw_rad, s.forward_velocity_mps, s.lateral_velocity_mps,
    s.yaw_rate_radps, s.desired_steering_rad, s.tire_steering_rad};
}

inline State state(const StateValues<double> & s) noexcept
{
  return {s[X], s[Y], s[Yaw], s[Forward], s[Left], s[YawRate], s[Desired], s[Tire]};
}

struct ScalarArithmetic
{
  static double sin(const double x) noexcept {return std::sin(x);}
  static double cos(const double x) noexcept {return std::cos(x);}
  static double clamp(const double x, const double lo, const double hi) noexcept
  {
    return std::clamp(x, lo, hi);
  }
  static double reverse_correction(
    const double u, const double acceleration, const double interval) noexcept
  {
    return u < 0 ? std::max(acceleration, -u / interval) : acceleration;
  }
  static double rolling(const double u, const double interval, const double limit) noexcept
  {
    return u == 0 ? 0 : -std::copysign(std::min(limit, std::abs(u) / interval), u);
  }
};

template<class Scalar, class Arithmetic>
BodyRates<Scalar> derivative(
  const StateValues<Scalar> & s, const Scalar & wire, const Parameters & p,
  const double force_interval_sec, const Arithmetic & arithmetic)
{
  const Scalar u = s[Forward], v = s[Left], r = s[YawRate];
  const double interval = std::max(force_interval_sec, p.minimum_force_interval_sec);
  Scalar acceleration = arithmetic.clamp(
    wire, -p.maximum_wire_deceleration_mps2, p.maximum_wire_acceleration_mps2);
  acceleration = arithmetic.reverse_correction(u, acceleration, interval);
  acceleration = arithmetic.clamp(
    acceleration, -p.maximum_wire_deceleration_mps2, p.maximum_wire_acceleration_mps2);
  const Scalar rolling = arithmetic.rolling(u, interval, p.rolling_mps2);
  Scalar fx{}, fy{}, moment{};
  for (const auto & wheel : p.wheels) {
    const Scalar angle = wheel.steerable ? s[Tire] : Scalar(0);
    const Scalar c = arithmetic.cos(angle), sn = arithmetic.sin(angle);
    const Scalar point_forward = u - r * Scalar(wheel.com_left_m);
    const Scalar point_left = v + r * Scalar(wheel.com_forward_m);
    const Scalar slip = -sn * point_forward + c * point_left;
    const Scalar lateral = Scalar(-wheel.cornering_per_sec) * slip;
    const Scalar drive = Scalar(wheel.traction_fraction) * (acceleration + rolling);
    const Scalar ax = c * drive - sn * lateral;
    const Scalar ay = sn * drive + c * lateral;
    fx = fx + ax;
    fy = fy + ay;
    moment = moment + (Scalar(wheel.com_forward_m) * ay - Scalar(wheel.com_left_m) * ax);
  }
  const Scalar reference_forward = u + r * Scalar(p.com_left_m);
  const Scalar reference_left = v - r * Scalar(p.com_forward_m);
  const Scalar c = arithmetic.cos(s[Yaw]), sn = arithmetic.sin(s[Yaw]);
  return {c * reference_forward - sn * reference_left,
    sn * reference_forward + c * reference_left, r,
    fx - Scalar(p.drag_per_sec) * u + r * v,
    fy - Scalar(p.drag_per_sec) * v - r * u,
    moment * Scalar(p.mass_kg) / p.yaw_inertia_kgm2 - Scalar(p.angular_drag_per_sec) * r};
}

template<class Scalar, class Arithmetic>
void update_tire(
  StateValues<Scalar> & s, const Parameters & p, const double dt,
  const Arithmetic & arithmetic)
{
  const Scalar wire_steering = s[Desired] * Scalar(p.steering_wire_gain);
  const Scalar demand = Scalar(p.tire_grip) * arithmetic.clamp(
    wire_steering, -p.maximum_wire_steering_rad, p.maximum_wire_steering_rad);
  const double alpha = dt / (p.tire_lag_sec + dt);
  s[Tire] = s[Tire] + arithmetic.clamp(
    Scalar(alpha) * (demand - s[Tire]), -p.tire_slew_radps * dt, p.tire_slew_radps * dt);
}

template<class Scalar>
StateValues<Scalar> body_increment(
  StateValues<Scalar> s, const BodyRates<Scalar> & rate, const double dt)
{
  for (std::size_t i = 0; i < rate.size(); ++i) {s[i] = s[i] + rate[i] * Scalar(dt);}
  return s;
}

}  // namespace multi_purpose_mpc_ros::mpcc_vehicle_model::kernel

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_VEHICLE_MODEL_KERNEL_HPP_
