#ifndef MULTI_PURPOSE_MPC_ROS__CANONICAL_RETAINED_WORLD_REVALIDATION_HPP_
#define MULTI_PURPOSE_MPC_ROS__CANONICAL_RETAINED_WORLD_REVALIDATION_HPP_

#include "multi_purpose_mpc_ros/canonical_retained_revalidation.hpp"
#include "multi_purpose_mpc_ros/mpc_stage_geometry.hpp"
#include "multi_purpose_mpc_ros/recovery_footprint.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::canonical_retained_world_revalidation
{

namespace retained = canonical_retained_revalidation;
namespace plan = canonical_execution_plan;

/// Course-frame endpoint tolerance shared by retained current-world proof and
/// the prediction reconstructed from that same proof.
inline constexpr double kCourseFrameIdentityToleranceM = 1e-9;

struct EmptyDynamicObstacleObservation
{
  std::uint64_t observation_generation{};
  std::uint64_t tube_id{};
  double observation_sec{};
  std::size_t active_vehicle_count{};
  bool current{false};
};

struct CurrentWorldProofRequest
{
  retained::CurrentExecutionProvenance current;
  double measured_course_progress_m{};
  double progress_continuity_tolerance_m{};
  std::vector<recovery_footprint::Pose2D> measured_to_control_path;
  recovery_footprint::Pose2D control_pose;
  std::vector<mpc_stage_geometry::CourseFrameKnot> course_frame_knots;
  EmptyDynamicObstacleObservation obstacles;
  double swept_step_m{};
};

enum class CurrentWorldProofReason
{
  Accepted,
  InvalidInput,
  WindowRejected,
  ProgressLiftRejected,
  ControlPoseIdentityMismatch,
  CourseFrameIdentityMismatch,
  ObstacleObservationUnavailable,
  ObstacleTubeIdentityMismatch,
  DynamicObstaclePresent,
  CourseFrameUnavailable,
  DelayPrefixBlocked,
  ConnectorBlocked,
  StagePathBlocked,
  ProofRejected,
};

const char * to_string(CurrentWorldProofReason reason) noexcept;

struct CurrentWorldProofResult
{
  CurrentWorldProofReason reason{CurrentWorldProofReason::InvalidInput};
  retained::RetainedExecutionProofReason proof_reason{
    retained::RetainedExecutionProofReason::InvalidPlan};
  std::optional<retained::RetainedExecutionProof> proof;
  std::size_t rejected_stage_index{};
};

/// Current dynamic-target tube used to re-certify a retained Follow plan.
/// Progress values are relative to the current MPCC course-progress origin;
/// elapsed time starts at zero and covers the complete retained window. The
/// separately sealed `current_target_gap_m` remains ego-relative.
struct FollowDynamicObstacleObservation
{
  std::string target_id;
  std::uint64_t observation_generation{};
  double observation_sec{};
  double current_target_gap_m{std::numeric_limits<double>::quiet_NaN()};
  double hard_gap_m{};
  std::vector<double> elapsed_time_sec;
  std::vector<double> target_progress_from_current_origin_m;
  std::uint64_t tube_id{};
  bool current{false};
};

struct FollowCurrentWorldProofRequest
{
  retained::CurrentExecutionProvenance current;
  double measured_course_progress_m{};
  double progress_continuity_tolerance_m{};
  std::vector<recovery_footprint::Pose2D> measured_to_control_path;
  recovery_footprint::Pose2D control_pose;
  std::vector<mpc_stage_geometry::CourseFrameKnot> course_frame_knots;
  FollowDynamicObstacleObservation target;
  double swept_step_m{};
};

enum class FollowCurrentWorldProofReason
{
  Accepted,
  InvalidInput,
  WindowRejected,
  ProgressLiftRejected,
  ControlPoseIdentityMismatch,
  CourseFrameIdentityMismatch,
  TargetObservationUnavailable,
  TargetIdentityMismatch,
  TargetTubeIdentityMismatch,
  TargetHorizonUnavailable,
  InitialHardGapViolation,
  CourseFrameUnavailable,
  DelayPrefixBlocked,
  ConnectorBlocked,
  StagePathBlocked,
  StageGapViolation,
  ProofRejected,
};

const char * to_string(FollowCurrentWorldProofReason reason) noexcept;

struct FollowCurrentWorldProofResult
{
  FollowCurrentWorldProofReason reason{
    FollowCurrentWorldProofReason::InvalidInput};
  retained::RetainedExecutionProofReason proof_reason{
    retained::RetainedExecutionProofReason::InvalidPlan};
  std::optional<retained::RetainedExecutionProof> proof;
  std::size_t rejected_stage_index{};
  double minimum_gap_m{std::numeric_limits<double>::infinity()};
};

/// Current target-dependent lateral corridor used to re-certify a retained
/// ShiftOut/Pass/Return plan. The corridor timestamps start at zero and cover
/// the complete remaining retained execution window. Target exclusion is
/// either encoded directly in the bounds or accompanied by the exact physical
/// release evidence which allowed the target bound to be removed.
struct OvertakeDynamicCorridorObservation
{
  std::string target_id;
  std::uint64_t observation_generation{};
  double observation_sec{};
  std::vector<double> elapsed_time_sec;
  std::vector<double> lateral_lower_m;
  std::vector<double> lateral_upper_m;
  bool target_exclusion_encoded{false};
  bool release_current_body_clear{false};
  bool release_prediction_valid{false};
  bool release_predicted_sweep_clear{false};
  std::uint64_t tube_id{};
  bool current{false};
};

enum class OvertakeTargetTubeIntersectionReason
{
  Accepted,
  InvalidInput,
  Infeasible,
};

const char * to_string(OvertakeTargetTubeIntersectionReason reason) noexcept;

struct OvertakeTargetTubeIntersectionRequest
{
  OvertakeDynamicCorridorObservation base_corridor;
  int pass_side_sign{};
  double required_center_separation_m{};
  std::vector<double> target_lateral_m;
  std::vector<bool> target_separation_active;
};

struct OvertakeTargetTubeIntersectionResult
{
  OvertakeTargetTubeIntersectionReason reason{
    OvertakeTargetTubeIntersectionReason::InvalidInput};
  std::optional<OvertakeDynamicCorridorObservation> corridor;
  std::size_t rejected_sample_index{};
};

/// Intersect a wall/trust corridor with the current target occupancy tube.
/// The result is immutable evidence that target exclusion is represented in
/// every time sample consumed by retained-plan world revalidation.
OvertakeTargetTubeIntersectionResult intersect_overtake_target_tube(
  const OvertakeTargetTubeIntersectionRequest & request) noexcept;

struct OvertakeCurrentWorldProofRequest
{
  retained::CurrentExecutionProvenance current;
  double measured_course_progress_m{};
  double measured_lateral_m{};
  double progress_continuity_tolerance_m{};
  std::vector<recovery_footprint::Pose2D> measured_to_control_path;
  recovery_footprint::Pose2D control_pose;
  std::vector<mpc_stage_geometry::CourseFrameKnot> course_frame_knots;
  OvertakeDynamicCorridorObservation corridor;
  double lateral_tolerance_m{};
  double required_wall_clearance_m{};
  double swept_step_m{};
};

enum class OvertakeCurrentWorldProofReason
{
  Accepted,
  InvalidInput,
  WindowRejected,
  ProgressLiftRejected,
  ControlPoseIdentityMismatch,
  CourseFrameIdentityMismatch,
  TargetObservationUnavailable,
  TargetIdentityMismatch,
  ExecutionSideMismatch,
  CorridorIdentityMismatch,
  CorridorHorizonUnavailable,
  TargetReleaseUncertified,
  InitialCorridorViolation,
  CourseFrameUnavailable,
  DelayPrefixBlocked,
  ConnectorBlocked,
  StagePathBlocked,
  StageCorridorViolation,
  ProofRejected,
};

const char * to_string(OvertakeCurrentWorldProofReason reason) noexcept;

struct OvertakeCurrentWorldProofResult
{
  OvertakeCurrentWorldProofReason reason{
    OvertakeCurrentWorldProofReason::InvalidInput};
  retained::RetainedExecutionProofReason proof_reason{
    retained::RetainedExecutionProofReason::InvalidPlan};
  std::optional<retained::RetainedExecutionProof> proof;
  std::size_t rejected_stage_index{};
  double minimum_corridor_reserve_m{std::numeric_limits<double>::infinity()};
};

std::uint64_t fingerprint_control_pose_path(
  const std::vector<recovery_footprint::Pose2D> & measured_to_control_path,
  const recovery_footprint::Pose2D & control_pose) noexcept;

std::uint64_t fingerprint_course_frame_window(
  const std::vector<mpc_stage_geometry::CourseFrameKnot> & knots) noexcept;

std::uint64_t fingerprint_empty_obstacle_observation(
  std::uint64_t observation_generation, double observation_sec) noexcept;

std::uint64_t fingerprint_follow_obstacle_observation(
  const FollowDynamicObstacleObservation & observation) noexcept;

std::uint64_t fingerprint_overtake_corridor_observation(
  const OvertakeDynamicCorridorObservation & observation) noexcept;

CurrentWorldProofResult build_current_world_retained_proof(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor,
  const CurrentWorldProofRequest & request,
  const recovery_footprint::OccupancyGrid & wall_grid,
  const recovery_footprint::FootprintExtents & footprint);

FollowCurrentWorldProofResult build_follow_current_world_retained_proof(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor,
  const FollowCurrentWorldProofRequest & request,
  const recovery_footprint::OccupancyGrid & wall_grid,
  const recovery_footprint::FootprintExtents & footprint);

OvertakeCurrentWorldProofResult build_overtake_current_world_retained_proof(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor,
  const OvertakeCurrentWorldProofRequest & request,
  const recovery_footprint::OccupancyGrid & wall_grid,
  const recovery_footprint::FootprintExtents & footprint);

}  // namespace multi_purpose_mpc_ros::canonical_retained_world_revalidation

#endif  // MULTI_PURPOSE_MPC_ROS__CANONICAL_RETAINED_WORLD_REVALIDATION_HPP_
