#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_DYNAMIC_PROOF_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_DYNAMIC_PROOF_HPP_

#include "multi_purpose_mpc_ros/recovery_footprint.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_proof
{

namespace recovery = recovery_footprint;

struct DynamicObstacle
{
  std::string id;
  recovery::CircleObstacle circle;
};

struct WorldObservation
{
  std::uint64_t generation{};
  double observed_sec{};
  std::vector<DynamicObstacle> obstacles;
  bool current{false};
};

/// Accumulator shared by fresh architecture replay and retained production
/// revalidation.  It owns no authority and can only reject a physical path.
struct Result
{
  bool valid{true};
  bool clear{true};
  std::string blocking_obstacle_id;
  recovery::DynamicClearanceRejectReason rejection_reason{
    recovery::DynamicClearanceRejectReason::None};
  std::string rejected_obstacle_id;
  double rejected_elapsed_sec{std::numeric_limits<double>::quiet_NaN()};
  recovery::Pose2D rejected_pose{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  double rejected_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  std::size_t checked_pose_count{};
  double minimum_clearance_m{std::numeric_limits<double>::infinity()};
  std::string minimum_clearance_obstacle_id;
  double minimum_clearance_elapsed_sec{
    std::numeric_limits<double>::quiet_NaN()};
  recovery::Pose2D minimum_clearance_pose{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::vector<recovery::DynamicClearanceSequence> obstacle_clearance;
};

bool observation_valid(const WorldObservation & observation) noexcept;

void observe_pose(
  const recovery::FootprintExtents & footprint,
  const recovery::Pose2D & pose,
  double elapsed_time_sec,
  const WorldObservation & observation,
  Result & result);

void observe_segment(
  const recovery::FootprintExtents & footprint,
  const recovery::Pose2D & start,
  const recovery::Pose2D & end,
  double start_time_sec,
  double end_time_sec,
  double swept_step_m,
  const WorldObservation & observation,
  Result & result);

void observe_timed_path(
  const recovery::FootprintExtents & footprint,
  const std::vector<recovery::Pose2D> & path,
  const std::vector<double> & elapsed_sec,
  double swept_step_m,
  const WorldObservation & observation,
  Result & result);

void finalize(
  const WorldObservation & observation,
  Result & result);

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_proof

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_DYNAMIC_PROOF_HPP_
