#ifndef MULTI_PURPOSE_MPC_ROS__CANONICAL_RETAINED_WORLD_REVALIDATION_HPP_
#define MULTI_PURPOSE_MPC_ROS__CANONICAL_RETAINED_WORLD_REVALIDATION_HPP_

#include "multi_purpose_mpc_ros/canonical_retained_revalidation.hpp"
#include "multi_purpose_mpc_ros/mpc_stage_geometry.hpp"
#include "multi_purpose_mpc_ros/recovery_footprint.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
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

std::uint64_t fingerprint_control_pose_path(
  const std::vector<recovery_footprint::Pose2D> & measured_to_control_path,
  const recovery_footprint::Pose2D & control_pose) noexcept;

std::uint64_t fingerprint_course_frame_window(
  const std::vector<mpc_stage_geometry::CourseFrameKnot> & knots) noexcept;

std::uint64_t fingerprint_empty_obstacle_observation(
  std::uint64_t observation_generation, double observation_sec) noexcept;

CurrentWorldProofResult build_current_world_retained_proof(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor,
  const CurrentWorldProofRequest & request,
  const recovery_footprint::OccupancyGrid & wall_grid,
  const recovery_footprint::FootprintExtents & footprint);

}  // namespace multi_purpose_mpc_ros::canonical_retained_world_revalidation

#endif  // MULTI_PURPOSE_MPC_ROS__CANONICAL_RETAINED_WORLD_REVALIDATION_HPP_
