#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::persistent_osqp
{

struct WarmStart
{
  Eigen::VectorXd primal;
  Eigen::VectorXd dual;
};

/// One contiguous, stage-major constraint block appended after the canonical
/// MPC dual layout. The producer must declare every appended block so that a
/// receding-horizon shift never guesses the temporal meaning of unknown rows.
struct DualStageBlockLayout
{
  std::size_t stage_count{};
  std::size_t rows_per_stage{};
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

/// Physical-unit provenance for the row that caused a post-solve constraint
/// rejection. Values are computed from the original A, l and u, never from a
/// solver-preconditioned problem.
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
};

/// Shift one successful MPC QP solution by one stage. The expected layout is
/// state[0..N], input[0..N-1] for primal variables and dynamics/state,
/// box(state,input), steering-rate for dual variables.
std::optional<WarmStart>
shift_mpc_warm_start(
  const WarmStart & previous, std::size_t horizon_steps,
  std::size_t state_dimension = 3U,
  std::size_t input_dimension = 2U,
  const std::vector<DualStageBlockLayout> & trailing_dual_stage_blocks = {}) noexcept;

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
    const std::optional<WarmStart> & warm_start = std::nullopt);

  void reset() noexcept;
  bool initialized() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace multi_purpose_mpc_ros::persistent_osqp
