#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::persistent_osqp
{

/// OSQP reports `solved inaccurate` when the residual is within ten times
/// the configured tolerance.  The physical-row certificate accepts the same
/// status, so any producer that reserves an exact actuator boundary must
/// reserve this maximum accepted multiplier as well.
inline constexpr double kSolvedInaccurateToleranceMultiplier = 10.0;

struct WarmStart
{
  Eigen::VectorXd primal;
  Eigen::VectorXd dual;
};

/// Diagonal change of variables used only inside the numerical solver.
/// `physical = physical_units_per_solver_unit * solver_coordinate`.
/// Callers, certificates and warm-start stores remain in physical units.
struct VariableCoordinateScaling
{
  Eigen::VectorXd physical_units_per_solver_unit;
};

/// Derive a unit scale from the finite physical box bounds of each variable.
/// A coordinate with no non-zero finite bound uses the neutral scale 1.0.
std::optional<VariableCoordinateScaling>
derive_box_variable_coordinate_scaling(
  const Eigen::VectorXd & lower_bound,
  const Eigen::VectorXd & upper_bound) noexcept;

/// One contiguous, stage-major constraint block appended after the canonical
/// MPC dual layout. The producer must declare every appended block so that a
/// receding-horizon shift never guesses the temporal meaning of unknown rows.
struct DualStageBlockLayout
{
  std::size_t stage_count{};
  std::size_t rows_per_stage{};
};

/// Explicit physical-stage layout for receding-horizon warm-start transport.
/// `rate_rows_per_stage` is one for the established curvature-input MPC and
/// zero for a steering-state/steering-rate-input formulation whose actuator
/// rate is already an input box. Keeping it explicit prevents a supposedly
/// generic state/input shift from silently requiring a legacy constraint row.
struct MpcWarmStartLayout
{
  std::size_t state_dimension{};
  std::size_t input_dimension{};
  std::size_t rate_rows_per_stage{1U};
  std::vector<DualStageBlockLayout> trailing_dual_stage_blocks;
};

struct SolveTelemetry
{
  bool setup_performed{false};
  bool update_performed{false};
  bool structural_rebuild{false};
  bool update_rebuild{false};
  bool warm_start_applied{false};
  bool warm_start_rejected{false};
  bool cold_reset_after_failure{false};
  bool maximum_iterations_reached{false};
  double setup_ms{};
  double update_ms{};
  double warm_start_ms{};
  double solve_ms{};
  double total_ms{};
  int iterations{};
  int status{};
  double objective_value{std::numeric_limits<double>::quiet_NaN()};
  double primal_residual{std::numeric_limits<double>::quiet_NaN()};
  double dual_residual{std::numeric_limits<double>::quiet_NaN()};
  int rho_updates{};
  double rho_estimate{std::numeric_limits<double>::quiet_NaN()};
  double absolute_tolerance{};
  double relative_tolerance{};
  int scaling_iterations{};
  bool scaled_termination{false};
  bool row_tolerance_preconditioned{false};
  bool variable_coordinate_scaled{false};
  double minimum_variable_scale{1.0};
  double maximum_variable_scale{1.0};
  double maximum_row_scale{1.0};
  double physical_constraint_scale{
    std::numeric_limits<double>::quiet_NaN()};
  double physical_global_tolerance{
    std::numeric_limits<double>::quiet_NaN()};
};

struct SolveResult
{
  Eigen::VectorXd primal;
  Eigen::VectorXd dual;
  int status{};
  double maximum_constraint_violation{};
  Eigen::VectorXd constraint_violation;
  Eigen::VectorXd constraint_tolerance;
  double maximum_normalized_constraint_violation{};
  int maximum_normalized_constraint_row{-1};
};

struct ConstraintResidualReport
{
  Eigen::VectorXd violation;
  Eigen::VectorXd tolerance;
  double maximum_absolute_violation{};
  double maximum_normalized_violation{};
  int maximum_normalized_row{-1};
};

/// Physical-unit provenance for the worst row in a rejected solver iterate.
/// This covers both a converged result rejected by the physical constraint
/// contract and a non-executable maximum-iteration diagnostic. Values are
/// computed from the original A, l and u, never from a solver-preconditioned
/// problem. The latter remains diagnostic only and is never exposed as a
/// SolveResult.
struct ConstraintFailureDiagnostic
{
  int row{-1};
  double value{};
  double projected{};
  double lower_bound{};
  double upper_bound{};
  double violation{};
  double tolerance{};
  double normalized_violation{};
};

/// Evaluate every constraint row in its own numerical scale. This report is
/// separate from OSQP's global infinity-norm termination test: one QP can
/// contain metres, radians, velocity and course-progress values, so a large
/// progress row must not enlarge the accepted error of a lateral safety row.
std::optional<ConstraintResidualReport> evaluate_constraint_residuals(
  const Eigen::SparseMatrix<double> & constraints,
  const Eigen::VectorXd & primal,
  const Eigen::VectorXd & lower_bound,
  const Eigen::VectorXd & upper_bound,
  double absolute_tolerance,
  double relative_tolerance,
  double tolerance_multiplier = 1.0) noexcept;

std::optional<ConstraintFailureDiagnostic> make_constraint_failure_diagnostic(
  const ConstraintResidualReport & report,
  const Eigen::VectorXd & constraint_values,
  const Eigen::VectorXd & lower_bound,
  const Eigen::VectorXd & upper_bound) noexcept;

struct SolveOutcome
{
  std::optional<SolveResult> result;
  /// Finite physical-coordinate iterate retained solely for offline failure
  /// diagnosis.  This is never a solved result and must not be converted to a
  /// command, warm-start Store entry or certified execution artifact without
  /// independently passing the architecture proof chain.
  std::optional<Eigen::VectorXd> rejected_primal;
  std::optional<ConstraintFailureDiagnostic> constraint_failure;
  std::string failure_detail;
  SolveTelemetry telemetry;
};

/// Optional exact row transformation applied before OSQP. The normalized
/// policy multiplies A, l and u by the same positive per-row factor, so it does
/// not change the physical feasible set or primal optimum. Returned duals stay
/// in the original physical constraint coordinates.
enum class ConstraintPreconditioningPolicy
{
  None,
  RowToleranceNormalized,
  /// Preserve the identical explicit physical row contract, then let OSQP
  /// apply its standard modified-Ruiz equilibration. Production use is limited
  /// to the wall-refinement QP class whose frozen failures established this as
  /// its canonical numerical owner; it is not a solve-time retry policy.
  RowToleranceNormalizedWithInternalEquilibration,
};

/// Physical-unit tolerances used to certify every original constraint row.
/// These values are captured before any row preconditioning changes the
/// solver-coordinate settings.
struct PhysicalConstraintTolerance
{
  double absolute{};
  double relative{};
};

/// Shift one successful MPC QP solution by the exact physical stage advance.
/// The expected layout is
/// state[0..N], input[0..N-1] for primal variables and dynamics/state,
/// box(state,input), steering-rate for dual variables.
std::optional<WarmStart>
shift_mpc_warm_start(
  const WarmStart & previous, std::size_t horizon_steps,
  std::size_t state_dimension = 3U,
  std::size_t input_dimension = 2U,
  const std::vector<DualStageBlockLayout> & trailing_dual_stage_blocks = {},
  std::size_t stage_advance = 1U) noexcept;

std::optional<WarmStart>
shift_mpc_warm_start(
  const WarmStart & previous, std::size_t horizon_steps,
  const MpcWarmStartLayout & layout,
  std::size_t stage_advance = 1U) noexcept;

class PersistentOsqpSolver
{
public:
  PersistentOsqpSolver();
  explicit PersistentOsqpSolver(ConstraintPreconditioningPolicy policy);
  ~PersistentOsqpSolver();

  PersistentOsqpSolver(const PersistentOsqpSolver &) = delete;
  PersistentOsqpSolver & operator=(const PersistentOsqpSolver &) = delete;
  PersistentOsqpSolver(PersistentOsqpSolver &&) noexcept;
  PersistentOsqpSolver & operator=(PersistentOsqpSolver &&) noexcept;

  SolveOutcome solve(
    Eigen::SparseMatrix<double> quadratic_cost,
    Eigen::SparseMatrix<double> constraints,
    const Eigen::VectorXd & linear_cost,
    const Eigen::VectorXd & lower_bound,
    const Eigen::VectorXd & upper_bound,
    const std::optional<WarmStart> & warm_start = std::nullopt,
    const std::optional<VariableCoordinateScaling> & variable_scaling =
    std::nullopt);

  void reset() noexcept;
  bool initialized() const noexcept;
  PhysicalConstraintTolerance physical_constraint_tolerance() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace multi_purpose_mpc_ros::persistent_osqp
