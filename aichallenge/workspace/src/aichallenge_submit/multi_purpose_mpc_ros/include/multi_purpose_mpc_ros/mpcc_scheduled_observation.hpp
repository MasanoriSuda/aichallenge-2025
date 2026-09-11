#pragma once

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_scheduled.hpp"
#include "multi_purpose_mpc_ros/v2x_overtake_core.hpp"

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled {

/// Use the canonical seconds representation of the exact appointed ns plus
/// configured delay. Multiplication by 1e-9 need not round like division by 1e9.
std::optional<double> publication_control_origin(
  const vehicle::PublishedInputProgram &program, std::size_t index,
  double prediction_delay_sec);

struct FollowCourseAnchor {
  std::string target_id;
  double observed_sec{};
  double target_path_progress_m{};
  double along_track_speed_mps{};
};

/// Anchor restricts branch association only. Gap and speed come from the fresh
/// Cartesian peer projection, never from relabeling an older gap's epoch.
v2x_overtake_core::ForwardCourseProjection project_follow_at_observation(
  const std::vector<v2x_overtake_core::CoursePoint> &course,
  v2x_overtake_core::ForwardCourseProjectionRequest current,
  const FollowCourseAnchor &anchor, double observed_sec);

/// Rebuild all point-state and predecessor fields from one actual public
/// observation. The optional packet is prospective only, at observation.now;
/// it never enters the actual history. Follow must be rebuilt by its producer.
std::optional<retained::Request> bind_current_observation(
  retained::Request seed, const vehicle::ObservationProvenance &observation,
  const ProgressFrame &frame,
  std::optional<vehicle::PublishedCommand> exact_next_packet = std::nullopt);

} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled
