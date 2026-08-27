#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_DYNAMIC_OBSTACLE_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_DYNAMIC_OBSTACLE_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_problem.hpp"

#include <Eigen/Dense>

#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_obstacle
{

struct StagePrediction
{
  bool valid{false};
  double target_progress_m{};
  double target_lateral_m{};
  double longitudinal_overlap_m{};
  double lateral_center_separation_m{};
};

struct Request
{
  bool active{false};
  int pass_side_sign{};
  std::vector<StagePrediction> stages;
  mpcc_rate_resolved_problem::AssemblyRequest wall_only_problem;
  Eigen::VectorXd wall_only_primal;
  double separation_tolerance_m{1e-6};
};

enum class Reason
{
  NotRequested,
  InvalidInput,
  NoPredictedEncounter,
  Applied,
};

const char * to_string(Reason reason) noexcept;

struct Result
{
  Reason reason{Reason::NotRequested};
  bool applied{false};
  int resolved_side_sign{};
  int first_pass_side_stage{-1};
  std::size_t stay_behind_row_count{};
  std::size_t pass_side_row_count{};
  std::size_t partial_escape_row_count{};
  int first_valid_stage{-1};
  double first_wall_only_progress_m{};
  double first_wall_only_effective_progress_m{};
  double first_wall_only_lateral_m{};
  double first_target_progress_m{};
  double first_target_lateral_m{};
  double first_stay_behind_margin_m{};
  double first_positive_side_margin_m{};
  double first_negative_side_margin_m{};
  std::optional<mpcc_rate_resolved_problem::AssemblyRequest> problem;
};

/// Convexifies the obstacle disjunction around a physically solved wall-only
/// trajectory.  With pass_side_sign==0, a trajectory which is already body
/// separated on one lateral side keeps that current physical homotopy even
/// when the nominal racing line crosses the peer later.  A wall-only path
/// which is coherently separated over the complete horizon also keeps its
/// homotopy; otherwise Cruise/Follow remains longitudinally behind.  With a
/// selected side, the optimized progress remains behind until a suffix has
/// achieved lateral body separation, after which that pass-side separation is
/// hard.  If the observed state is already inside both full-separation and
/// stay-behind bounds, the demonstrated wall-only homotopy instead supplies
/// the reachable partial escape envelope until full separation is reached.
/// The envelope may initially decrease when steering/yaw-response lag makes
/// an instantaneous non-decrease physically unreachable; it is never made
/// stricter than the wall-only trajectory which witnesses the branch.
/// This is the QP form of the same initial-overlap escape contract used by the
/// current-world physical verifier; it does not weaken a newly clear state.
/// Longitudinal classification and rows use the physical progress coordinate
/// theta + e_lag, never virtual progress theta alone.
Result refine(const Request & request) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_obstacle

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_DYNAMIC_OBSTACLE_HPP_
