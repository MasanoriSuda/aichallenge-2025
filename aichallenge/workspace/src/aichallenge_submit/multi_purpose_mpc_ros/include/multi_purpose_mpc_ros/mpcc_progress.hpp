#pragma once

#include <Eigen/Dense>

#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_progress
{

struct Config
{
  double minimum_reference_speed_mps{0.5};
  double minimum_frenet_denominator{0.20};
  double minimum_stage_dt_sec{0.01};
  double maximum_stage_dt_sec{0.25};
  double trust_region_backward_m{12.0};
  double trust_region_forward_m{2.0};
  double lag_weight{5000.0};
  double terminal_lag_weight{2500.0};
  double progress_reward_weight{2000.0};
  double terminal_progress_reward_weight{5000.0};
  bool extended_dynamics_enabled{false};
  double extended_lag_state_bound_m{3.0};
  // Extended dynamics have different state units and numerical scaling from
  // the established 3-state MPCC. Keep their tracking weights explicit rather
  // than inheriting one scale factor for both stage and terminal costs.
  double extended_lateral_tracking_weight{500.0};
  double extended_heading_tracking_weight{5000.0};
  double extended_terminal_lateral_tracking_weight{1500.0};
  double extended_terminal_heading_tracking_weight{5000.0};
  double extended_lag_weight{100.0};
  double extended_terminal_lag_weight{150.0};
  double extended_progress_tracking_weight{2.0};
  double extended_terminal_progress_tracking_weight{5.0};
  double extended_progress_reward_weight{4.0};
  double extended_terminal_progress_reward_weight{10.0};
  double extended_failure_cooldown_sec{0.75};
  double extended_mode_handoff_sec{0.15};
  double stage_velocity_weight{8.0};
  double committed_stage_velocity_weight{24.0};
  double terminal_velocity_weight{12.0};
  double committed_terminal_velocity_weight{45.0};
  double acceleration_weight{0.5};
  double virtual_progress_weight{0.5};
  double acceleration_rate_weight{0.8};
  double curvature_rate_weight{0.2};
  double virtual_progress_rate_weight{0.1};
  int rti_sqp_iterations{2};
  double rti_sqp_mixing{0.65};
  bool conditional_refinement_enabled{true};
  double refinement_minimum_bound_reserve_m{0.12};
  double refinement_lateral_defect_m{0.08};
  double refinement_heading_defect_rad{0.04};
  double refinement_curvature_radpm{0.08};
  double refinement_start_deadline_ms{12.0};
  double refinement_cold_entry_skip_sec{0.30};
  std::size_t refinement_wall_cache_miss_skip_threshold{1U};
};

struct StageDistanceResolution
{
  std::vector<double> distance_m;
  std::size_t normalized_stage_count{};
  double minimum_stage_distance_m{};
};

/// Resolve stage distances for temporal progress dynamics. A finite zero-length
/// stage can appear at a circular path seam when the closure point is repeated.
/// Such stages are lifted only to the distance implied by the minimum speed and
/// minimum integration period. Negative and non-finite distances remain invalid.
std::optional<StageDistanceResolution> resolve_stage_distances(
  const std::vector<double> & raw_stage_distance_m, const Config & config) noexcept;

/// Detect a course-progress discontinuity that makes an earlier MPCC warm-start
/// unsafe to reinterpret, notably the positive-to-zero wrap at a lap boundary.
bool progress_origin_discontinuous(
  double previous_progress_m, double current_progress_m,
  double maximum_continuous_step_m) noexcept;

struct LinearizationRequest
{
  double reference_lateral_m{};
  double reference_heading_rad{};
  double reference_progress_m{};
  double reference_speed_mps{};
  double reference_path_curvature_radpm{};
  double reference_input_curvature_radpm{};
  double stage_distance_m{};
  Config config;
};

struct Linearization
{
  Eigen::Matrix3d state_matrix{Eigen::Matrix3d::Identity()};
  Eigen::Matrix<double, 3, 2> input_matrix{Eigen::Matrix<double, 3, 2>::Zero()};
  // QP equality convention:
  //   -x[k+1] + A*x[k] + B*u[k] = equality_offset
  Eigen::Vector3d equality_offset{Eigen::Vector3d::Zero()};
  double stage_dt_sec{};
};

/// Linearize the temporal Frenet kinematic model around one stage reference.
/// State is [e_y, e_psi, s], input is [v, kappa].
std::optional<Linearization> linearize_temporal_frenet(
  const LinearizationRequest & request) noexcept;

inline constexpr int kExtendedStateDimension = 5;
inline constexpr int kExtendedInputDimension = 3;
inline constexpr int kExtendedLateralIndex = 0;
inline constexpr int kExtendedLagIndex = 1;
inline constexpr int kExtendedHeadingIndex = 2;
inline constexpr int kExtendedVelocityIndex = 3;
inline constexpr int kExtendedProgressIndex = 4;
inline constexpr int kExtendedAccelerationIndex = 0;
inline constexpr int kExtendedCurvatureIndex = 1;
inline constexpr int kExtendedVirtualProgressSpeedIndex = 2;

struct ExtendedLinearizationRequest
{
  double reference_lateral_m{};
  double reference_lag_m{};
  double reference_heading_rad{};
  double reference_velocity_mps{};
  double reference_progress_m{};
  double reference_acceleration_mps2{};
  double reference_path_curvature_radpm{};
  double reference_input_curvature_radpm{};
  double reference_virtual_progress_speed_mps{};
  double stage_distance_m{};
  Config config;
};

struct ExtendedLinearization
{
  Eigen::Matrix<double, kExtendedStateDimension, kExtendedStateDimension>
  state_matrix{
    Eigen::Matrix<double, kExtendedStateDimension, kExtendedStateDimension>::Identity()};
  Eigen::Matrix<double, kExtendedStateDimension, kExtendedInputDimension>
  input_matrix{
    Eigen::Matrix<double, kExtendedStateDimension, kExtendedInputDimension>::Zero()};
  Eigen::Matrix<double, kExtendedStateDimension, 1> equality_offset{
    Eigen::Matrix<double, kExtendedStateDimension, 1>::Zero()};
  double stage_dt_sec{};
};

/// Linearize the extended temporal Frenet model used by the overtake MPCC.
/// State is [e_y, e_lag, e_psi, v, theta], input is [a, kappa, v_theta].
std::optional<ExtendedLinearization> linearize_extended_temporal_frenet(
  const ExtendedLinearizationRequest & request) noexcept;

struct VelocityHorizonRequest
{
  std::vector<double> reference_velocity_mps;
  std::vector<double> hard_cap_velocity_mps;
  bool committed_pass{false};
  Config config;
};

struct VelocityHorizon
{
  std::vector<double> reference_velocity_mps;
  std::vector<double> hard_cap_velocity_mps;
  std::vector<double> stage_weight;
  double terminal_target_velocity_mps{};
  double terminal_weight{};
};

/// Keep the soft speed objective separate from the safety cap.  committed Pass
/// raises only the objective weight; it never relaxes a hard stage limit.
std::optional<VelocityHorizon> resolve_velocity_horizon(
  const VelocityHorizonRequest & request) noexcept;

/// Convert an accepted 5x3 solution into the established 3x2 layout consumed
/// by prediction, physical wall validation and command post-processing. The
/// extended theta state is local to progress_origin_m and is restored here.
std::optional<Eigen::VectorXd> convert_extended_solution_to_legacy(
  const Eigen::VectorXd & extended_primal, int horizon_size,
  double progress_origin_m) noexcept;

/// Rebase a shifted extended warm-start from the previous local progress
/// origin to the current one. Only theta state elements are modified.
bool rebase_extended_progress_warm_start(
  Eigen::VectorXd & extended_primal, int horizon_size,
  double previous_progress_origin_m, double current_progress_origin_m) noexcept;

class ExtendedSolverCircuitBreaker
{
public:
  bool active(double now_sec) const noexcept;
  void record_failure(double now_sec, double cooldown_sec) noexcept;
  void record_success() noexcept;
  void reset() noexcept;
  double disabled_until_sec() const noexcept;

private:
  double disabled_until_sec_{-std::numeric_limits<double>::infinity()};
};

struct ExtendedModeHandoffResolution
{
  double velocity_mps{};
  double blend_ratio{1.0};
  bool active{false};
};

/// Preserve longitudinal command continuity when execution changes between
/// the extended and established MPCC formulations. The resolved command is
/// always clipped to the current cycle's hard velocity bounds; a newly lower
/// front-risk cap therefore takes effect immediately.
class ExtendedModeHandoff
{
public:
  std::optional<ExtendedModeHandoffResolution> resolve_velocity(
    bool extended_mode, double now_sec, double desired_velocity_mps,
    double current_lower_mps, double current_upper_mps,
    double handoff_duration_sec) noexcept;
  void reset() noexcept;

private:
  std::optional<bool> previous_extended_mode_;
  double last_output_velocity_mps_{std::numeric_limits<double>::quiet_NaN()};
  double transition_source_velocity_mps_{std::numeric_limits<double>::quiet_NaN()};
  double transition_start_sec_{-std::numeric_limits<double>::infinity()};
};

/// Build an unwrapped stage progress reference from the measured progress and
/// the existing ReferencePath segment distances. The result has N+1 states.
std::optional<std::vector<double>> build_progress_reference(
  double measured_progress_m, const std::vector<double> & stage_distance_m) noexcept;

struct ProgressBounds
{
  double lower_m{};
  double upper_m{};
};

/// Resolve an asymmetric trust region. The measured progress is always kept in
/// the lower envelope so a hard speed cap cannot make the QP infeasible merely
/// because the nominal progress horizon moved ahead.
std::optional<ProgressBounds> resolve_progress_bounds(
  double measured_progress_m, double reference_progress_m,
  const Config & config) noexcept;

struct ProgressCost
{
  double quadratic_weight{};
  double linear_coefficient{};
};

/// Cost convention is 0.5*w*s^2 + q*s. This represents
/// 0.5*w*(s-s_ref)^2 - reward*s up to an irrelevant constant.
std::optional<ProgressCost> resolve_progress_cost(
  double reference_progress_m, bool terminal, const Config & config) noexcept;

/// Form the next RTI-SQP linearization point without replacing a feasible
/// trajectory in one jump. alpha=1 adopts the QP solution directly.
std::optional<Eigen::VectorXd> damp_rti_sqp_iterate(
  const Eigen::VectorXd & linearization_point,
  const Eigen::VectorXd & qp_solution, double alpha) noexcept;

struct RtiColdLoadRequest
{
  bool progress_execution_context_active{false};
  double now_sec{};
  double mission_start_sec{std::numeric_limits<double>::quiet_NaN()};
  double cold_entry_skip_sec{};
  std::size_t wall_cache_miss_count{};
  std::size_t wall_cache_miss_skip_threshold{};
};

/// Identify the bounded startup/cache-rebuild cycles where a second QP solve
/// should yield to the first feasible result. A zero duration or miss
/// threshold disables only that trigger.
bool rti_refinement_cold_load_active(const RtiColdLoadRequest & request) noexcept;

struct RtiRefinementRequest
{
  bool progress_mode_active{false};
  int configured_iterations{1};
  bool conditional_refinement_enabled{true};
  bool cold_load_active{false};
  double minimum_lateral_bound_reserve_m{
    std::numeric_limits<double>::infinity()};
  double lateral_defect_m{};
  double heading_defect_rad{};
  double maximum_curvature_radpm{};
  double elapsed_ms{};
  double minimum_bound_reserve_threshold_m{};
  double lateral_defect_threshold_m{};
  double heading_defect_threshold_rad{};
  double curvature_threshold_radpm{};
  double refinement_start_deadline_ms{};
};

enum class RtiRefinementDecision
{
  Disabled,
  Refine,
  SkipColdLoad,
  SkipCondition,
  SkipDeadline,
  Invalid,
};

/// Decide whether a second RTI-SQP solve is worth spending the remaining
/// control-cycle budget.  The first feasible QP remains authoritative when
/// refinement is skipped or later fails.
RtiRefinementDecision resolve_rti_refinement(
  const RtiRefinementRequest & request) noexcept;

struct ExecutionTrajectory
{
  std::vector<double> path_distance_m;
  std::vector<double> lateral_m;
  std::vector<double> progress_m;
  double minimum_lateral_bound_reserve_m{};
};

enum class ExecutionTrajectoryRejection
{
  None,
  InvalidInput,
  InvalidPathDistance,
  InvalidLateralBounds,
  NonFiniteState,
  ProgressRegressed,
  LateralOutOfBounds,
  EmptyTrajectory,
};

struct ExecutionTrajectoryDiagnostic
{
  ExecutionTrajectoryRejection rejection{ExecutionTrajectoryRejection::None};
  int stage{-1};
};

const char * execution_trajectory_rejection_name(
  ExecutionTrajectoryRejection rejection) noexcept;

/// Extract the state stages which were actually admitted by the MPCC QP.
/// The QP layout is [N+1 x (e_y,e_psi,s), N x (v,kappa)]. Stage zero is the
/// measured equality, so the returned execution path contains stages 1..N.
std::optional<ExecutionTrajectory> extract_execution_trajectory(
  const Eigen::VectorXd & primal, int horizon_size,
  const std::vector<double> & path_distance_m,
  const std::vector<double> & lateral_lower_m,
  const std::vector<double> & lateral_upper_m,
  double bound_tolerance_m = 1e-5,
  ExecutionTrajectoryDiagnostic * diagnostic = nullptr) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_progress
