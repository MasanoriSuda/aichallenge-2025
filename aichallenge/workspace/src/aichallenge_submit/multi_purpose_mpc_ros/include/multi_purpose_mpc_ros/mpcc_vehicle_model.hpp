#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_VEHICLE_MODEL_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_VEHICLE_MODEL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace multi_purpose_mpc_ros::mpcc_vehicle_model
{

/// Empirical planar model at the effective actuation boundary. Transport and
/// command application epochs belong to the immutable caller's input history.
/// A nominal rollout does not guarantee future contact or packet delivery.
struct Wheel
{
  double com_forward_m{};
  double com_left_m{};
  double traction_fraction{};
  double cornering_per_sec{};
  bool steerable{};
};

struct Parameters
{
  std::string profile_id;
  double mass_kg{};
  double yaw_inertia_kgm2{};
  double com_forward_m{};
  double com_left_m{};
  double rolling_mps2{};
  double drag_per_sec{};
  double angular_drag_per_sec{};
  double maximum_wire_acceleration_mps2{};
  double maximum_wire_deceleration_mps2{};
  double sleep_speed_mps{};
  double minimum_force_interval_sec{};
  double tire_lag_sec{};
  double tire_slew_radps{};
  double tire_grip{};
  double steering_wire_gain{};
  double maximum_wire_steering_rad{};
  double maximum_step_sec{};
  /// Empirical settled-road assumption, not a future grounded observation.
  bool nominal_settled_contact{};
  std::array<Wheel, 4> wheels;
};

/// Pose at base_link; velocity at COM in forward/left axes. Angular quantities
/// are ROS counterclockwise. Desired steering precedes the single wire gain;
/// physical tire steering is a distinct measured/predicted state.
struct State
{
  double x_m{};
  double y_m{};
  double yaw_rad{};
  double forward_velocity_mps{};
  double lateral_velocity_mps{};
  double yaw_rate_radps{};
  double desired_steering_rad{};
  double tire_steering_rad{};
};

struct Input
{
  double wire_acceleration_mps2{};
  double desired_steering_rate_radps{};
};

struct BodyDerivative
{
  double x_mps{};
  double y_mps{};
  double yaw_radps{};
  double forward_acceleration_mps2{};
  double lateral_acceleration_mps2{};
  double yaw_acceleration_radps2{};
};

struct Transition
{
  State state;
  std::size_t substeps{};
  bool nominal_rest_applied{};
};

std::size_t integration_steps(double duration_sec, double maximum_step_sec) noexcept;

bool valid(const Parameters & parameters) noexcept;
bool finite(const State & state) noexcept;
/// Includes every coefficient, coordinate offset, actuator conversion and rest
/// assumption. Invalid profiles have no identity; labels alone never suffice.
std::uint64_t fingerprint(const Parameters & parameters) noexcept;

/// Moving-mode force law at a supplied physical tire angle. Rest/launch and
/// discrete actuator evolution are owned by advance(), never a publisher clamp.
std::optional<BodyDerivative> derivative(
  const State & state, double wire_acceleration_mps2,
  const Parameters & parameters, double force_interval_sec) noexcept;

/// Same native transition for optimization, physical proof, Stop, retained
/// cursors and the committed-input prefix. Invalid/nonfinite requests fail.
/// Positive wire input always leaves the nominal rest mode.
std::optional<Transition> advance(
  State state, const Input & input, const Parameters & parameters,
  double duration_sec) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_vehicle_model

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_VEHICLE_MODEL_HPP_
