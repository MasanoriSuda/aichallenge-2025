#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

namespace multi_purpose_mpc_ros::persistent_osqp
{

struct WarmStart
{
  Eigen::VectorXd primal;
  Eigen::VectorXd dual;
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
};

struct SolveOutcome
{
  std::optional<SolveResult> result;
  std::string failure_detail;
  SolveTelemetry telemetry;
};

/// Shift one successful MPC QP solution by one stage. The expected layout is
/// state[0..N], input[0..N-1] for primal variables and dynamics/state,
/// box(state,input), steering-rate for dual variables.
std::optional<WarmStart>
shift_mpc_warm_start(
  const WarmStart & previous, std::size_t horizon_steps,
  std::size_t state_dimension = 3U,
  std::size_t input_dimension = 2U) noexcept;

class PersistentOsqpSolver
{
public:
  PersistentOsqpSolver();
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
