#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_RETAINED_REVALIDATION_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_RETAINED_REVALIDATION_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_certified_plan.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_dynamic_proof.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"
#include "multi_purpose_mpc_ros/mpcc_latest_state_feedback.hpp"

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
namespace race = race_mpcc_foundation;

using DynamicObstacle = mpcc_rate_resolved_dynamic_proof::DynamicObstacle;
using DynamicWorldObservation = mpcc_rate_resolved_dynamic_proof::WorldObservation;

/// Convert the V2X planner's forbidden ego-center distance into the radius of
/// the peer-only circle consumed by the physical footprint verifier.  The
/// planner distance already contains the ego body half width; passing it
/// directly as a peer radius would inflate the ego body twice.
std::optional<double> resolve_peer_circle_radius(
  double forbidden_ego_center_distance_m,
  const recovery::FootprintExtents & ego_footprint,
  double peer_uncertainty_margin_m) noexcept;

/// Current Follow target in the same course-progress frame used to build the
/// current seven-state semantic problem.  This is not retained solver state: it
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

struct FollowTargetObservationBuildRequest
{
  std::string target_id;
  std::uint64_t observation_generation{};
  double observed_sec{};
  double current_target_gap_m{std::numeric_limits<double>::quiet_NaN()};
  double current_ego_progress_offset_m{
    std::numeric_limits<double>::quiet_NaN()};
  double hard_gap_m{std::numeric_limits<double>::quiet_NaN()};
  double target_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> stage_duration_sec;
  bool current{false};
};

/// Build fresh Follow evidence from an intent-independent current-world
/// target projection.  The resulting horizon has no solver authority; it is
/// consumed only when revalidating an already published Follow artifact.
std::optional<FollowTargetObservation> build_follow_target_observation(
  const FollowTargetObservationBuildRequest & request) noexcept;

enum class ExecutionClockKind
{
  Unknown,
  BootstrapCandidate,
  TimeAlignedCandidate,
  PublishedPlan,
};

const char * to_string(ExecutionClockKind kind) noexcept;

/// Causal execution time is not certificate age.  A bootstrap candidate has
/// no executed predecessor and starts at cursor zero.  A moving unpublished
/// successor is spliced at the suffix corresponding to the current control
/// origin; its skipped prefix receives no authority and the suffix must still
/// pass the current physical-state, actuator, wall and dynamic-world connector
/// proofs.  A published plan instead advances from the artifact-local cursor
/// at its first exact publisher join.  Publication time alone is insufficient
/// when the first command came from a time-aligned candidate suffix.
struct ExecutionClock
{
  ExecutionClockKind kind{ExecutionClockKind::Unknown};
  double first_published_control_origin_sec{
    std::numeric_limits<double>::quiet_NaN()};
  double first_published_artifact_elapsed_sec{
    std::numeric_limits<double>::quiet_NaN()};
};

artifact::Cursor resolve_execution_cursor(
  const artifact::ExecutionArtifact & execution,
  double current_control_origin_sec,
  const ExecutionClock & clock) noexcept;

struct Request
{
  std::shared_ptr<const certified::CertifiedPlan> plan;
  std::uint64_t decision_id{};
  double now_sec{};
  double control_origin_sec{};
  ExecutionClock execution_clock;
  contract::ControlIntent current_intent{contract::ControlIntent::Unknown};
  /// Continuous physical along-course position projected to control_origin_sec.
  /// This is theta + lag, not the cumulative progress of a discrete associated
  /// waypoint.
  double control_origin_physical_progress_m{};
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
  /// Velocity projected to control_origin_sec by the same latency predictor
  /// which produced control_pose.
  double control_origin_speed_mps{};
  /// Physical steering estimated at request.now_sec from the latest measured
  /// report and the command which was already committed before this cycle.
  double current_time_steering_rad{};
  /// Physical steering projected to control_origin_sec.  It is diagnostic
  /// observation provenance only; the seven-state command trajectory and
  /// command reachability do not originate from this value.
  double current_steering_rad{};
  /// Effective yaw-producing steering projected to control_origin_sec.
  double current_response_steering_rad{};
  /// Last steering command successfully serialized to the actuator.  This is
  /// the integrated command-state origin and owns publication-to-publication
  /// slew reachability.  Observed physical steering instead contributes to
  /// the measured-to-control response prediction.
  double previous_published_steering_rad{};
  /// Actual age of previous_published_steering_rad at current evaluation.
  /// Callback overrun and asynchronous solve time make the nominal controller
  /// period an invalid substitute for this causal publication duration.
  double previous_published_command_age_sec{};
  /// Exact moving-Stop lateral law used by both terminal proof and publisher.
  race_mpcc_foundation::StopPathTrackingPolicy stop_lateral_policy;
  double minimum_acceleration_mps2{};
  double maximum_acceleration_mps2{};
};

enum class Reason
{
  Accepted,
  MissingPlan,
  InvalidPlan,
  ExecutionClockInvalid,
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
  ControlPathInvalid,
  DelayPrefixBlocked,
  ConnectorBlocked,
  ContinuationRejected,
  ContinuationWallBlocked,
  TerminalContingencyUnavailable,
  Count,
};

const char * to_string(Reason reason) noexcept;

/// Static-wall extent observed while revalidating a retained artifact. A
/// publisher-interval-only result is diagnostic until an exact certified stop
/// or successor suffix exists; it must not receive production authority.
enum class StaticWallProofScope
{
  FullSuffix,
  PublisherIntervalPrefix,
};

const char * to_string(StaticWallProofScope scope) noexcept;

/// Dynamic-world extent observed while revalidating a retained artifact. A
/// future obstacle is a replanning obligation, but one publisher interval
/// alone is not an executable safety certificate without a certified terminal
/// suffix.
enum class DynamicObstacleProofScope
{
  FullSuffix,
  PublisherIntervalPrefix,
};

const char * to_string(DynamicObstacleProofScope scope) noexcept;

struct Proof
{
  std::shared_ptr<const certified::CertifiedPlan> plan;
  /// True when the source artifact's first steering sample could not join the
  /// actually serialized predecessor and this proof instead owns the exact
  /// latest-state feedback connection.  Such a proof is a stateless
  /// current-world bundle: it may command the proved actuation, but it must
  /// never claim that the unmodified source artifact was published.
  bool latest_state_feedback_bundle{false};
  /// True when the exact artifact-time stage did not contain one complete
  /// publisher interval and the proof instead joined the immediately
  /// following sealed control stage from the current physical state.  The
  /// skipped residual is never executed and the source artifact must not be
  /// promoted as an unmodified executed plan.
  bool publication_stage_advanced{false};
  bool stateless_current_world_bundle() const noexcept
  {
    return latest_state_feedback_bundle || publication_stage_advanced;
  }
  std::uint64_t decision_id{};
  std::uint64_t obstacle_generation{};
  double observed_sec{};
  double observation_origin_sec{};
  double control_origin_sec{};
  double prediction_delay_sec{};
  artifact::Cursor cursor;
  /// Current-world joined actuation.  Acceleration, steering-rate, steering
  /// and progress-rate remain the immutable artifact controls.  Velocity is
  /// a state, not an actuator input, and is therefore re-anchored to the fresh
  /// control-origin observation used by continuation certification.
  artifact::Actuation actuation;
  artifact::PredictedState expected_current_state;
  recovery::Pose2D expected_current_pose;
  /// Course-frame progress state (theta) from the immutable artifact.
  double expected_absolute_progress_m{};
  /// Physical along-course position (theta + lag) from the artifact.
  double expected_physical_progress_m{};
  double lifted_control_origin_physical_progress_m{};
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
  StaticWallProofScope static_wall_scope{StaticWallProofScope::FullSuffix};
  DynamicObstacleProofScope dynamic_obstacle_scope{
    DynamicObstacleProofScope::FullSuffix};
  mpcc_rate_resolved_physical_adapter::ContinuationProofScope
  continuation_scope{
    mpcc_rate_resolved_physical_adapter::ContinuationProofScope::FullSuffix};
  std::size_t proved_control_stage_count{};
  std::size_t static_wall_checked_pose_count{};
  std::size_t dynamic_checked_pose_count{};
  double minimum_dynamic_clearance_m{
    std::numeric_limits<double>::infinity()};
  std::uint64_t follow_target_observation_generation{};
  std::size_t follow_checked_state_count{};
  double follow_minimum_gap_m{std::numeric_limits<double>::infinity()};
  /// A partial normal prefix may own exactly one publisher interval only when
  /// this current-decision Stop trajectory is independently certified.
  bool terminal_stop_certified{false};
  std::size_t terminal_stop_static_checked_pose_count{};
  std::size_t terminal_stop_dynamic_checked_pose_count{};
  double terminal_stop_minimum_dynamic_clearance_m{
    std::numeric_limits<double>::infinity()};
  race_mpcc_foundation::ExactPhysicalExecutionTrajectory
  continuation_trajectory;
  race_mpcc_foundation::ExactPhysicalExecutionTrajectory
  terminal_stop_trajectory;
  std::vector<mpcc_rate_resolved_physical_adapter::StopContingencyResult::
    ActuationSample> terminal_stop_actuation_samples;
  std::size_t terminal_stop_publisher_interval_sample_count{};
  std::vector<double> continuation_stage_end_velocity_mps;
  std::vector<double> continuation_stage_end_steering_rad;
};

struct Result
{
  Reason reason{Reason::MissingPlan};
  artifact::CursorReason cursor_reason{artifact::CursorReason::InvalidArtifact};
  artifact::ActuationReason actuation_reason{
    artifact::ActuationReason::InvalidArtifact};
  ExecutionClockKind execution_clock_kind{ExecutionClockKind::Unknown};
  double first_published_control_origin_sec{
    std::numeric_limits<double>::quiet_NaN()};
  double first_published_artifact_elapsed_sec{
    std::numeric_limits<double>::quiet_NaN()};
  double cursor_elapsed_sec{std::numeric_limits<double>::quiet_NaN()};
  bool publication_stage_advanced{false};
  std::size_t source_control_stage_index{};
  std::size_t command_control_stage_index{};
  double publication_stage_advance_sec{};
  std::string blocking_obstacle_id;
  std::size_t dynamic_checked_pose_count{};
  double minimum_dynamic_clearance_m{
    std::numeric_limits<double>::infinity()};
  std::uint64_t follow_target_observation_generation{};
  std::size_t follow_checked_state_count{};
  double follow_minimum_gap_m{std::numeric_limits<double>::infinity()};
  bool terminal_stop_attempted{false};
  bool terminal_stop_certified{false};
  mpcc_rate_resolved_physical_adapter::StopContingencyRejectReason
  terminal_stop_reason{
    mpcc_rate_resolved_physical_adapter::StopContingencyRejectReason::
    InvalidArtifact};
  race_mpcc_foundation::ExactPhysicalExecutionTrajectoryReason
  terminal_stop_exact_reason{
    race_mpcc_foundation::ExactPhysicalExecutionTrajectoryReason::Accepted};
  int terminal_stop_rejected_sample{-1};
  double terminal_stop_publisher_interval_end_steering_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double terminal_stop_final_steering_rad{
    std::numeric_limits<double>::quiet_NaN()};
  recovery::PathClearanceResult terminal_stop_path_clearance;
  std::string terminal_stop_blocking_obstacle_id;
  std::size_t terminal_stop_dynamic_checked_pose_count{};
  double terminal_stop_minimum_dynamic_clearance_m{
    std::numeric_limits<double>::infinity()};
  std::size_t terminal_stop_follow_checked_state_count{};
  double terminal_stop_follow_minimum_gap_m{
    std::numeric_limits<double>::infinity()};
  double expected_absolute_progress_m{
    std::numeric_limits<double>::quiet_NaN()};
  double expected_physical_progress_m{
    std::numeric_limits<double>::quiet_NaN()};
  double expected_lateral_m{std::numeric_limits<double>::quiet_NaN()};
  double expected_lag_m{std::numeric_limits<double>::quiet_NaN()};
  double expected_heading_offset_rad{
    std::numeric_limits<double>::quiet_NaN()};
  recovery::Pose2D control_pose;
  recovery::Pose2D expected_current_pose;
  double control_pose_error_m{std::numeric_limits<double>::quiet_NaN()};
  double control_yaw_error_rad{std::numeric_limits<double>::quiet_NaN()};
  double lifted_control_origin_physical_progress_m{
    std::numeric_limits<double>::quiet_NaN()};
  double progress_difference_m{
    std::numeric_limits<double>::quiet_NaN()};
  double progress_continuity_tolerance_m{
    std::numeric_limits<double>::quiet_NaN()};
  double current_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double control_origin_speed_mps{
    std::numeric_limits<double>::quiet_NaN()};
  double expected_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double current_time_steering_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double current_steering_rad{std::numeric_limits<double>::quiet_NaN()};
  double current_response_steering_rad{
    std::numeric_limits<double>::quiet_NaN()};
  bool current_control_state_available{false};
  artifact::PredictedState current_control_state;
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
  bool feedback_shadow_attempted{false};
  mpcc_latest_state_feedback::Reason feedback_shadow_reason{
    mpcc_latest_state_feedback::Reason::InvalidInput};
  double feedback_shadow_steering_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double feedback_shadow_correction_rad{
    std::numeric_limits<double>::quiet_NaN()};
  mpcc_rate_resolved_physical_adapter::ContinuationRejectReason
  feedback_shadow_continuation_reason{
    mpcc_rate_resolved_physical_adapter::ContinuationRejectReason::
    InvalidArtifact};
  race_mpcc_foundation::ExactPhysicalExecutionTrajectoryReason
  feedback_shadow_exact_reason{
    race_mpcc_foundation::ExactPhysicalExecutionTrajectoryReason::Accepted};
  bool feedback_shadow_continuation_available{false};
  Reason feedback_shadow_proof_reason{Reason::MissingPlan};
  bool feedback_shadow_proof_available{false};
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
  StaticWallProofScope static_wall_scope{StaticWallProofScope::FullSuffix};
  DynamicObstacleProofScope dynamic_obstacle_scope{
    DynamicObstacleProofScope::FullSuffix};
  mpcc_rate_resolved_physical_adapter::ContinuationProofScope
  continuation_scope{
    mpcc_rate_resolved_physical_adapter::ContinuationProofScope::FullSuffix};
  race_mpcc_foundation::ExactPhysicalExecutionTrajectoryReason
  continuation_exact_reason{
    race_mpcc_foundation::ExactPhysicalExecutionTrajectoryReason::Accepted};
  std::size_t proved_control_stage_count{};
  recovery::PathClearanceResult publisher_interval_path_clearance;
  recovery::PathClearanceResult continuation_path_clearance;
  mpcc_rate_resolved_physical_adapter::ContinuationRejectReason
  continuation_reason{
    mpcc_rate_resolved_physical_adapter::ContinuationRejectReason::
    InvalidArtifact};
  std::optional<Proof> proof;
};

/// Revalidate only the current-world join to an immutable, already physically
/// certified suffix.  This function deliberately cannot produce a command.
Result evaluate(const Request & request);

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_retained_revalidation

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_RETAINED_REVALIDATION_HPP_
