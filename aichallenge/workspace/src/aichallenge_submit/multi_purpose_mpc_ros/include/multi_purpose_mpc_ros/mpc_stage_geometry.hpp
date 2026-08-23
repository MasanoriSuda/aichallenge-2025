#ifndef MULTI_PURPOSE_MPC_ROS__MPC_STAGE_GEOMETRY_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPC_STAGE_GEOMETRY_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpc_stage_geometry
{

struct Point2d
{
  double x_m{};
  double y_m{};
};

struct Stage
{
  int transition_from_waypoint{};
  int state_waypoint{};
  double transition_distance_m{};
  double cumulative_distance_m{};
};

struct Geometry
{
  bool valid{false};
  int tracking_waypoint{};
  bool circular{false};
  std::vector<Stage> stages;
  std::string reject_reason;
};

/// One reference-path pose whose progress belongs to the same local,
/// unwrapped coordinate frame as an MPCC theta state.
struct CourseFrameKnot
{
  double progress_m{};
  double x_m{};
  double y_m{};
  double heading_rad{};
  int waypoint{};
};

struct CourseFrameSample
{
  double progress_m{};
  double x_m{};
  double y_m{};
  double heading_rad{};
  int lower_waypoint{};
  int upper_waypoint{};
  double interpolation_ratio{};
};

/// One solved Frenet state on the same unwrapped progress axis as
/// CourseFrameKnot.
struct FrenetTrajectoryState
{
  double progress_m{};
  double lateral_m{};
  double lag_m{};
  double heading_offset_rad{};
};

/// Dense world sample reconstructed by interpolating in Frenet/course space,
/// rather than drawing a straight world-frame chord between sparse states.
struct CourseFollowingPose
{
  double x_m{};
  double y_m{};
  double heading_rad{};
  std::size_t destination_state_index{};
  double segment_ratio{};
};

/// Build the single stage-index contract shared by dynamics, lateral bounds,
/// wall validation and execution certificates.  Stage zero is the state after
/// the transition tracking_waypoint -> tracking_waypoint + 1.
Geometry build(
  const std::vector<Point2d> & path, int tracking_waypoint,
  std::size_t stage_count, bool circular) noexcept;

/// Interpolate the course frame at the progress actually solved by MPCC.
/// Knots must be finite and strictly increasing. Queries outside the supplied
/// provenance window fail closed instead of silently using a different course
/// section or extrapolating through a curve.
std::optional<CourseFrameSample> sample_course_frame(
  const std::vector<CourseFrameKnot> & knots,
  double query_progress_m,
  double query_tolerance_m = 1e-9) noexcept;

/// Densify a solved Frenet trajectory on the course frame. The first state is
/// included once; every later segment is sampled at no more than
/// maximum_progress_step_m in solved progress. States must be finite and
/// nondecreasing in progress.
std::optional<std::vector<CourseFollowingPose>> sample_course_following_trajectory(
  const std::vector<CourseFrameKnot> & course_knots,
  const std::vector<FrenetTrajectoryState> & states,
  double maximum_progress_step_m) noexcept;

}  // namespace multi_purpose_mpc_ros::mpc_stage_geometry

#endif  // MULTI_PURPOSE_MPC_ROS__MPC_STAGE_GEOMETRY_HPP_
