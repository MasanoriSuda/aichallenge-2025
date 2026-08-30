#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_CERTIFIED_STOP_SUCCESSOR_OBSERVATION_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_CERTIFIED_STOP_SUCCESSOR_OBSERVATION_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_production_adapter.hpp"

#include <cstdint>
#include <limits>

namespace multi_purpose_mpc_ros::mpcc_certified_stop_successor_observation
{

namespace production = mpcc_rate_resolved_production_adapter;

struct Published
{
  production::CertifiedStopSuccessorEvidence evidence;
  double publication_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct CurrentControlOrigin
{
  std::uint64_t decision_id{};
  bool state_available{false};
  double observation_origin_sec{std::numeric_limits<double>::quiet_NaN()};
  double control_origin_sec{std::numeric_limits<double>::quiet_NaN()};
  double x_m{std::numeric_limits<double>::quiet_NaN()};
  double y_m{std::numeric_limits<double>::quiet_NaN()};
  double yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double speed_mps{std::numeric_limits<double>::quiet_NaN()};
  /// Canonical command-control-origin steering used by the next problem.
  double steering_rad{std::numeric_limits<double>::quiet_NaN()};
  /// Physical report at the observation timestamp, before latency projection.
  double current_time_steering_rad{std::numeric_limits<double>::quiet_NaN()};
  /// Yaw-response steering projected to the command control origin.
  double response_control_origin_steering_rad{
    std::numeric_limits<double>::quiet_NaN()};
  /// Last physical steering command which crossed the publisher boundary.
  double previous_published_steering_rad{
    std::numeric_limits<double>::quiet_NaN()};
};

enum class Reason
{
  Sampled,
  InvalidIdentity,
  InvalidShape,
  InvalidCurrentState,
  TimeOutsideSuccessor,
  Count,
};

const char * to_string(Reason reason) noexcept;

struct Result
{
  Reason reason{Reason::InvalidIdentity};
  double successor_elapsed_sec{std::numeric_limits<double>::quiet_NaN()};
  double publisher_boundary_sec{std::numeric_limits<double>::quiet_NaN()};
  double publication_age_sec{std::numeric_limits<double>::quiet_NaN()};
  std::size_t lower_sample_index{};
  std::size_t upper_sample_index{};
  double interpolation_alpha{};
  double expected_x_m{std::numeric_limits<double>::quiet_NaN()};
  double expected_y_m{std::numeric_limits<double>::quiet_NaN()};
  double expected_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double expected_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double expected_steering_rad{std::numeric_limits<double>::quiet_NaN()};
  double position_error_m{std::numeric_limits<double>::quiet_NaN()};
  double yaw_error_rad{std::numeric_limits<double>::quiet_NaN()};
  double speed_error_mps{std::numeric_limits<double>::quiet_NaN()};
  /// Command-control-origin error. This preserves the original field meaning.
  double steering_error_rad{std::numeric_limits<double>::quiet_NaN()};
  double current_time_steering_error_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double response_control_origin_steering_error_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double previous_published_steering_error_rad{
    std::numeric_limits<double>::quiet_NaN()};
};

Result evaluate(
  const Published & published,
  const CurrentControlOrigin & current) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_certified_stop_successor_observation

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_CERTIFIED_STOP_SUCCESSOR_OBSERVATION_HPP_
