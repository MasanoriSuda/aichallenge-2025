#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_RETAINED_REVALIDATION_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_RETAINED_REVALIDATION_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_certified_plan.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_retained_revalidation
{

namespace artifact = mpcc_rate_resolved_execution_artifact;
namespace certified = mpcc_rate_resolved_certified_plan;
namespace contract = mpcc_execution_contract;
namespace physical = mpcc_rate_resolved_physical_wall;
namespace recovery = recovery_footprint;

struct DynamicObstacle
{
  std::string id;
  recovery::CircleObstacle circle;
};

struct DynamicWorldObservation
{
  std::uint64_t generation{};
  double observed_sec{};
  std::vector<DynamicObstacle> obstacles;
  bool current{false};
};

/// Current Follow target in the same course-progress frame used to build the
/// current six-state semantic problem.  This is not retained solver state: it
/// is fresh world evidence which must re-certify the retained suffix.
struct FollowTargetObservation
{
  std::string target_id;
  std::uint64_t observation_generation{};
  double observed_sec{};
  double current_target_gap_m{std::numeric_limits<double>::quiet_NaN()};
  double hard_gap_m{std::numeric_limits<double>::quiet_NaN()};
  double target_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> elapsed_time_sec;
  std::vector<double> target_progress_from_current_origin_m;
  bool current{false};
};

struct Request
{
  std::shared_ptr<const certified::CertifiedPlan> plan;
  std::uint64_t decision_id{};
  double now_sec{};
  double control_origin_sec{};
  contract::ControlIntent current_intent{contract::ControlIntent::Unknown};
  double measured_course_progress_m{};
  double path_length_m{};
  double progress_continuity_tolerance_m{};
  bool circular{false};
  std::vector<recovery::Pose2D> measured_to_control_path;
  std::vector<double> measured_to_control_elapsed_sec;
  recovery::Pose2D control_pose;
  std::shared_ptr<const recovery::OccupancyGrid> current_wall_grid;
  recovery::FootprintExtents current_footprint;
  DynamicWorldObservation obstacles;
  std::optional<FollowTargetObservation> follow_target;
  double current_speed_mps{};
  /// Physical steering estimated at request.now_sec from the latest measured
  /// report and the command which was already committed before this cycle.
  double current_time_steering_rad{};
  /// Physical steering projected to control_origin_sec if the previously
  /// committed command remains held.  This is diagnostic provenance and the
  /// fresh problem's nominal initial state, not the predecessor of the next
  /// serialized command.
  double current_steering_rad{};
  /// Last steering command successfully serialized to the actuator.  Command
  /// slew is a publication-to-publication contract and must not be inferred
  /// from either observed physical steering value above.
  double previous_published_steering_rad{};
  double publication_interval_sec{};
  double minimum_acceleration_mps2{};
  double maximum_acceleration_mps2{};
};

enum class Reason
{
  Accepted,
  MissingPlan,
  InvalidPlan,
  CursorUnavailable,
  IntentMismatch,
  DynamicObservationUnavailable,
  DynamicObservationInvalid,
  FollowTargetObservationUnavailable,
  FollowTargetObservationInvalid,
  FollowTargetIdentityMismatch,
  FollowTargetHorizonUnavailable,
  FollowInitialHardGapViolation,
  FollowStageGapViolation,
  DynamicPathInvalid,
  DynamicPathBlocked,
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
  double observation_origin_sec{};
  double control_origin_sec{};
  double prediction_delay_sec{};
  artifact::Cursor cursor;
  artifact::Actuation actuation;
  artifact::PredictedState expected_current_state;
  recovery::Pose2D expected_current_pose;
  double expected_absolute_progress_m{};
  double lifted_measured_progress_m{};
  long lap_offset{};
  double current_time_steering_rad{};
  double previous_published_steering_rad{};
  double steering_difference_rad{};
  double maximum_steering_step_rad{};
  double reachable_steering_lower_rad{};
  double reachable_steering_upper_rad{};
  double steering_reachability_duration_sec{};
  double velocity_difference_mps{};
  double reachable_velocity_lower_mps{};
  double reachable_velocity_upper_mps{};
  double velocity_reachability_duration_sec{};
  std::size_t delay_checked_pose_count{};
  std::size_t connector_checked_pose_count{};
  std::size_t dynamic_checked_pose_count{};
  double minimum_dynamic_clearance_m{
    std::numeric_limits<double>::infinity()};
  std::uint64_t follow_target_observation_generation{};
  std::size_t follow_checked_state_count{};
  double follow_minimum_gap_m{std::numeric_limits<double>::infinity()};
};

struct Result
{
  Reason reason{Reason::MissingPlan};
  artifact::CursorReason cursor_reason{artifact::CursorReason::InvalidArtifact};
  artifact::ActuationReason actuation_reason{
    artifact::ActuationReason::InvalidArtifact};
  double cursor_elapsed_sec{std::numeric_limits<double>::quiet_NaN()};
  std::string blocking_obstacle_id;
  std::size_t dynamic_checked_pose_count{};
  double minimum_dynamic_clearance_m{
    std::numeric_limits<double>::infinity()};
  std::uint64_t follow_target_observation_generation{};
  std::size_t follow_checked_state_count{};
  double follow_minimum_gap_m{std::numeric_limits<double>::infinity()};
  double expected_absolute_progress_m{
    std::numeric_limits<double>::quiet_NaN()};
  double lifted_measured_progress_m{
    std::numeric_limits<double>::quiet_NaN()};
  double progress_difference_m{
    std::numeric_limits<double>::quiet_NaN()};
  double progress_continuity_tolerance_m{
    std::numeric_limits<double>::quiet_NaN()};
  double current_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double expected_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double current_time_steering_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double current_steering_rad{std::numeric_limits<double>::quiet_NaN()};
  double previous_published_steering_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double expected_steering_rad{std::numeric_limits<double>::quiet_NaN()};
  double steering_difference_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double maximum_steering_step_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double reachable_steering_lower_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double reachable_steering_upper_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double steering_reachability_duration_sec{
    std::numeric_limits<double>::quiet_NaN()};
  double velocity_difference_mps{
    std::numeric_limits<double>::quiet_NaN()};
  double reachable_velocity_lower_mps{
    std::numeric_limits<double>::quiet_NaN()};
  double reachable_velocity_upper_mps{
    std::numeric_limits<double>::quiet_NaN()};
  double velocity_reachability_duration_sec{
    std::numeric_limits<double>::quiet_NaN()};
  recovery::PathClearanceResult delay_path_clearance;
  recovery::PathClearanceResult connector_path_clearance;
  std::optional<Proof> proof;
};

/// Revalidate only the current-world join to an immutable, already physically
/// certified suffix.  This function deliberately cannot produce a command.
Result evaluate(const Request & request);

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_retained_revalidation

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_RETAINED_REVALIDATION_HPP_
