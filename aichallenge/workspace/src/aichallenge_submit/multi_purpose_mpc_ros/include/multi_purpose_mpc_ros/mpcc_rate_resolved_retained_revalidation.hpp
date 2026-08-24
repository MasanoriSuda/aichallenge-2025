#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_RETAINED_REVALIDATION_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_RETAINED_REVALIDATION_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_certified_plan.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_retained_revalidation
{

namespace artifact = mpcc_rate_resolved_execution_artifact;
namespace certified = mpcc_rate_resolved_certified_plan;
namespace contract = mpcc_execution_contract;
namespace physical = mpcc_rate_resolved_physical_wall;
namespace recovery = recovery_footprint;

struct EmptyDynamicObstacleObservation
{
  std::uint64_t generation{};
  double observed_sec{};
  std::size_t active_vehicle_count{};
  bool current{false};
};

struct Request
{
  std::shared_ptr<const certified::CertifiedPlan> plan;
  std::uint64_t decision_id{};
  double now_sec{};
  contract::ControlIntent current_intent{contract::ControlIntent::Unknown};
  double measured_course_progress_m{};
  double path_length_m{};
  double progress_continuity_tolerance_m{};
  bool circular{false};
  std::vector<recovery::Pose2D> measured_to_control_path;
  recovery::Pose2D control_pose;
  std::shared_ptr<const recovery::OccupancyGrid> current_wall_grid;
  recovery::FootprintExtents current_footprint;
  EmptyDynamicObstacleObservation obstacles;
  double current_speed_mps{};
  double current_steering_rad{};
  double minimum_acceleration_mps2{};
  double maximum_acceleration_mps2{};
  double publication_interval_sec{};
};

enum class Reason
{
  Accepted,
  MissingPlan,
  InvalidPlan,
  CursorUnavailable,
  IntentMismatch,
  DynamicObservationUnavailable,
  DynamicObstaclePresent,
  StaticWorldMismatch,
  InvalidCurrentState,
  ProgressLiftRejected,
  CourseFrameUnavailable,
  ActuationRejected,
  SteeringUnreachable,
  VelocityUnreachable,
  ControlPathInvalid,
  DelayPrefixBlocked,
  ConnectorBlocked,
  Count,
};

const char * to_string(Reason reason) noexcept;

struct Proof
{
  std::shared_ptr<const certified::CertifiedPlan> plan;
  std::uint64_t decision_id{};
  std::uint64_t obstacle_generation{};
  double observed_sec{};
  artifact::Cursor cursor;
  artifact::Actuation actuation;
  artifact::PredictedState expected_current_state;
  recovery::Pose2D expected_current_pose;
  double expected_absolute_progress_m{};
  double lifted_measured_progress_m{};
  long lap_offset{};
  double steering_difference_rad{};
  double maximum_steering_step_rad{};
  double velocity_difference_mps{};
  double reachable_velocity_lower_mps{};
  double reachable_velocity_upper_mps{};
  std::size_t delay_checked_pose_count{};
  std::size_t connector_checked_pose_count{};
};

struct Result
{
  Reason reason{Reason::MissingPlan};
  artifact::CursorReason cursor_reason{artifact::CursorReason::InvalidArtifact};
  artifact::ActuationReason actuation_reason{
    artifact::ActuationReason::InvalidArtifact};
  std::optional<Proof> proof;
};

/// Revalidate only the current-world join to an immutable, already physically
/// certified suffix.  This function deliberately cannot produce a command.
Result evaluate(const Request & request);

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_retained_revalidation

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_RETAINED_REVALIDATION_HPP_
