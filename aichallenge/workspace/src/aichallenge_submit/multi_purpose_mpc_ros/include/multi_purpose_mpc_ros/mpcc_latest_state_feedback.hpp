#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_LATEST_STATE_FEEDBACK_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_LATEST_STATE_FEEDBACK_HPP_

#include <limits>
#include <optional>

namespace multi_purpose_mpc_ros::mpcc_latest_state_feedback
{

enum class Reason
{
  InvalidInput,
  EmptyEnvelope,
  PreparedCommandReachable,
  ProjectedToReachableEnvelope,
};

const char * to_string(Reason reason) noexcept;

struct Request
{
  double previous_published_steering_rad{};
  double prepared_steering_rad{};
  double maximum_abs_steering_rad{};
  double maximum_abs_steering_rate_radps{};
  double publication_age_sec{};
  double physical_certificate_tolerance{};
};

struct Result
{
  Reason reason{Reason::InvalidInput};
  double lower_rad{std::numeric_limits<double>::quiet_NaN()};
  double upper_rad{std::numeric_limits<double>::quiet_NaN()};
  double prepared_steering_rad{std::numeric_limits<double>::quiet_NaN()};
  double feedback_steering_rad{std::numeric_limits<double>::quiet_NaN()};
  double correction_rad{std::numeric_limits<double>::quiet_NaN()};

  bool available() const noexcept
  {
    return reason == Reason::PreparedCommandReachable ||
           reason == Reason::ProjectedToReachableEnvelope;
  }

  bool projected() const noexcept
  {
    return reason == Reason::ProjectedToReachableEnvelope;
  }
};

Result solve(const Request & request) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_latest_state_feedback

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_LATEST_STATE_FEEDBACK_HPP_
