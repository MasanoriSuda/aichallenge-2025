#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_ON_TRAJECTORY_CONNECTOR_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_ON_TRAJECTORY_CONNECTOR_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_certified_plan.hpp"

#include <cstdint>
#include <limits>
#include <memory>

namespace multi_purpose_mpc_ros::mpcc_on_trajectory_connector
{

namespace artifact = mpcc_rate_resolved_execution_artifact;
namespace certified = mpcc_rate_resolved_certified_plan;

enum class Reason
{
  Accepted,
  InvalidRequest,
  InvalidParent,
  InvalidCandidate,
  IncompatibleModel,
  ParentClockInvalid,
  SwitchBeforeCandidateOrigin,
  ParentCursorUnavailable,
  CandidateCursorUnavailable,
  StateMismatch,
};

const char * to_string(Reason reason) noexcept;

struct State
{
  double lateral_m{};
  double lag_m{};
  double heading_offset_rad{};
  double velocity_mps{};
  double absolute_progress_m{};
  double steering_rad{};
  double response_steering_rad{};
};

/// Observation-only comparison of one new asynchronous candidate with the
/// last actually published certified trajectory.  This type contains no
/// command or production-authority representation.
struct Request
{
  std::shared_ptr<const certified::CertifiedPlan> parent;
  std::shared_ptr<const certified::CertifiedPlan> candidate;
  double parent_first_published_control_origin_sec{
    std::numeric_limits<double>::quiet_NaN()};
  double parent_first_published_artifact_elapsed_sec{
    std::numeric_limits<double>::quiet_NaN()};
  double switch_control_origin_sec{
    std::numeric_limits<double>::quiet_NaN()};
  double path_length_m{
    std::numeric_limits<double>::quiet_NaN()};
  bool circular{false};
};

struct Result
{
  Reason reason{Reason::InvalidRequest};
  std::uint64_t parent_sequence{};
  std::uint64_t candidate_sequence{};
  artifact::CursorReason parent_cursor_reason{
    artifact::CursorReason::InvalidArtifact};
  artifact::CursorReason candidate_cursor_reason{
    artifact::CursorReason::InvalidArtifact};
  double parent_elapsed_sec{std::numeric_limits<double>::quiet_NaN()};
  double candidate_elapsed_sec{std::numeric_limits<double>::quiet_NaN()};
  State parent_state;
  State candidate_state;
  double lateral_difference_m{std::numeric_limits<double>::quiet_NaN()};
  double lag_difference_m{std::numeric_limits<double>::quiet_NaN()};
  double heading_difference_rad{std::numeric_limits<double>::quiet_NaN()};
  double velocity_difference_mps{std::numeric_limits<double>::quiet_NaN()};
  double progress_difference_m{std::numeric_limits<double>::quiet_NaN()};
  double steering_difference_rad{std::numeric_limits<double>::quiet_NaN()};
  double response_steering_difference_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double position_tolerance_m{std::numeric_limits<double>::quiet_NaN()};
  double model_tolerance{std::numeric_limits<double>::quiet_NaN()};

  bool accepted() const noexcept
  {
    return reason == Reason::Accepted;
  }
};

Result evaluate(const Request & request) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_on_trajectory_connector

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_ON_TRAJECTORY_CONNECTOR_HPP_
