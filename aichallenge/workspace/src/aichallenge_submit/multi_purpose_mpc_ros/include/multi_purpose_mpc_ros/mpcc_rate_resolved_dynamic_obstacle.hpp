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
  /// Compatibility separations for callers without an immutable physical
  /// replay world.  When PhysicalSeparationGeometry is present, every branch
  /// classification and row uses its oriented support instead.
  double longitudinal_overlap_m{};
  double lateral_center_separation_m{};
};

/// Exact asymmetric ego body and peer circle used by the nonlinear dynamic
/// certificate.  Every production dynamic-obstacle disjunct is derived from
/// this same geometry; scalar StagePrediction separation is only a
/// compatibility source when an immutable replay world is unavailable.
struct PhysicalSeparationGeometry
{
  double ego_front_extent_m{};
  double ego_rear_extent_m{};
  double ego_left_extent_m{};
  double ego_right_extent_m{};
  double ego_margin_m{};
  double opponent_radius_m{};
};

struct Request
{
  bool active{false};
  int pass_side_sign{};
  /// Exact disjunction contract. When present, stages before this index use
  /// the complete stay-behind disjunct and this stage onward uses the complete
  /// selected-side disjunct. Audit candidates and the bounded late production
  /// member use this same representation.
  std::optional<int> forced_first_pass_side_stage;
  /// Optional first stage whose physical disjunct is longitudinally ahead.
  /// It must follow a forced side stage; `horizon` means no ahead row yet.
  std::optional<int> forced_first_ahead_stage;
  /// Candidate-D continuation only. Zero places each forced row on the
  /// wall-only witness and one restores the exact physical disjunct. The
  /// bounded production member always uses one.
  std::optional<double> forced_constraint_fraction;
  /// Shadow/offline candidate-E only. Stages before start use the exact
  /// stay-behind row, stages start..full interpolate a coupled diagonal
  /// supporting row, and later stages use exact selected-side separation.
  std::optional<int> forced_diagonal_start_stage;
  std::optional<int> forced_diagonal_full_side_stage;
  std::optional<PhysicalSeparationGeometry>
    forced_physical_separation_geometry;
  /// Current-world physical geometry used by production to derive a complete
  /// behind-to-side separating diagonal when neither axis-aligned disjunct is
  /// initially reachable. Without this certificate source, refinement keeps
  /// only complete axis-aligned disjuncts and must not weaken them to the
  /// obstacle-free wall witness.
  std::optional<PhysicalSeparationGeometry>
    physical_separation_geometry;
  std::vector<StagePrediction> stages;
  /// Physically solved witness used only to classify the reachable convex
  /// obstacle branch. Its progress trust buckets must not implicitly become
  /// constraints of a later coupled solve.
  mpcc_rate_resolved_problem::AssemblyRequest wall_only_problem;
  Eigen::VectorXd wall_only_primal;
  /// Optional compatible broad problem which receives the obstacle rows.
  /// When absent, rows are applied to wall_only_problem for callers which do
  /// not need coupled wall/obstacle refinement.
  std::optional<mpcc_rate_resolved_problem::AssemblyRequest>
    constraint_target_problem;
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
  bool forced_transition_applied{false};
  double forced_constraint_fraction{1.0};
  int resolved_side_sign{};
  int first_pass_side_stage{-1};
  std::size_t stay_behind_row_count{};
  std::size_t pass_side_row_count{};
  std::size_t ahead_row_count{};
  std::size_t diagonal_row_count{};
  bool physical_axis_support_applied{false};
  bool physical_diagonal_guidance_applied{false};
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
/// current-world physical verifier.  Once the current physical state has full
/// lateral separation on an explicitly selected side, that acquired homotopy
/// remains hard even if the obstacle-free wall witness crosses back later.
/// A separated middle prediction alone never establishes this ownership.
/// Longitudinal classification and rows use the physical progress coordinate
/// theta + e_lag, never virtual progress theta alone.
Result refine(const Request & request) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_obstacle

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_DYNAMIC_OBSTACLE_HPP_
