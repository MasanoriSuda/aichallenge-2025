#pragma once

#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"

#include <array>

namespace multi_purpose_mpc_ros::mpcc_current_wall_interval_observation
{

constexpr std::size_t kMaximumClearRuns = 32U;

/// Original producer inputs/results only; this observation grants no authority.
struct Observation
{
  std::uint64_t decision_id{};
  double ros_sec{};
  std::uint64_t prior_moving_decision_id{};
  double prior_moving_pose_sec{};
  double prior_moving_velocity_mps{};
  int intent{};
  int waypoint{};
  std::shared_ptr<const recovery_footprint::OccupancyGrid> grid;
  recovery_footprint::FootprintExtents footprint;
  recovery_footprint::Pose2D temporal_pose;
  recovery_footprint::Pose2D reference_pose;
  recovery_footprint::Pose2D query_pose;
  double projected_lag_m{};
  double projected_lateral_m{};
  double projected_heading_rad{};
  double applied_lag_m{};
  double spatial_lateral_m{};
  double spatial_heading_rad{};
  double lower_m{};
  double upper_m{};
  double clearance_m{};
  double sample_step_m{};
  double boundary_guard_m{};
  recovery_footprint::LateralClearRun sampling_anchor;
  bool runs_valid{false};
  std::size_t checked_pose_count{};
  std::size_t actual_run_count{};
  std::array<recovery_footprint::LateralClearRun, kMaximumClearRuns> clear_runs{};
  recovery_footprint::LateralClearIntervalResult selected;
  std::filesystem::path output_root{"mpcc_architecture_snapshots/current-wall-interval"};
};

/// Bounded scalar copies only; never copy/recompute the grid or geometry query.
void capture_result(
  Observation &, const recovery_footprint::LateralClearRunsResult &,
  const recovery_footprint::LateralClearIntervalResult &) noexcept;

/// Only call from an owned background task, once per live MPC owner.
mpcc_architecture_snapshot::RecordResult record(const Observation &) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_current_wall_interval_observation
