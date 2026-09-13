#pragma once

#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/stuck_recovery_core.hpp"

#include <vector>

namespace multi_purpose_mpc_ros::mpcc_recovery_direction_observation
{

/// Diagnostic capacity only. Dropped observations never change candidate evaluation.
constexpr std::size_t kMaximumTrials = 256U;

struct Trial
{
  std::string phase;
  recovery_footprint::ReversePrimitive primitive{};
  recovery_footprint::ReverseRolloutParameters rollout;
  recovery_footprint::ContactEscapePolicy contact_policy{};
  double minimum_contact_reduction_ratio{};
  recovery_footprint::FeasibilityResult result_without_rollout;
  std::size_t rollout_pose_count{};
  std::optional<recovery_footprint::Pose2D> endpoint;
  /// Diagnostic evaluation of the original course predicate, not a selection decision.
  stuck_recovery::RecoveryCourseProgressResolution course;
};

struct Observation
{
  std::uint64_t decision_id{};
  double ros_sec{};
  double steady_sec{};
  double signed_speed_mps{};
  std::shared_ptr<const recovery_footprint::OccupancyGrid> grid;
  recovery_footprint::FootprintExtents footprint;
  recovery_footprint::Pose2D pose;
  double lateral_error_m{};
  double heading_error_rad{};
  double course_activation_lateral_m{};
  double course_worsening_tolerance_m{};
  bool reverse_only{false};
  bool forward_probe_allowed{false};
  bool prefer_forward{false};
  bool full_rollout{false};
  bool current_footprint_clear{false};
  int wall_region{};
  int selected_direction{};
  bool static_clear{false};
  bool v2x_clear{false};
  bool information_complete{false};
  bool complete{true};
  std::size_t attempted_trial_count{};
  std::vector<Trial> trials;
  std::filesystem::path output_root{"mpcc_architecture_snapshots/recovery-direction"};
};

/// Save compact immutable inputs/results only. Never copy the rollout or map here.
void append_trial(
  Observation &, const char * phase, recovery_footprint::ReversePrimitive,
  const recovery_footprint::ReverseRolloutParameters &,
  recovery_footprint::ContactEscapePolicy, double minimum_contact_reduction_ratio,
  const recovery_footprint::FeasibilityResult &) noexcept;

/// Hashing, binary map serialization and atomic I/O must run outside control.
/// One node-owned future records the first failed CheckClearance direction only.
mpcc_architecture_snapshot::RecordResult record(const Observation &) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_recovery_direction_observation
