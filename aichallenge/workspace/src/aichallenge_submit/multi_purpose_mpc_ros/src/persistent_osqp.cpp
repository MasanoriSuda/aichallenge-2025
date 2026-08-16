#include <multi_purpose_mpc_ros/persistent_osqp.hpp>

#include <osqp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

namespace multi_purpose_mpc_ros::persistent_osqp
{
namespace
{

using SteadyClock = std::chrono::steady_clock;

double elapsed_ms(const SteadyClock::time_point start)
{
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - start)
         .count();
}

struct CscDeleter
{
  void operator()(csc * matrix) const noexcept {c_free(matrix);}
};

struct WorkspaceDeleter
{
  void operator()(OSQPWorkspace * workspace) const noexcept
  {
    if (workspace != nullptr) {
      static_cast<void>(osqp_cleanup(workspace));
    }
  }
};

bool sparse_values_are_finite(const Eigen::SparseMatrix<double> & matrix)
{
  for (int index = 0; index < matrix.nonZeros(); ++index) {
    if (!std::isfinite(matrix.valuePtr()[index])) {
      return false;
    }
  }
  return true;
}

bool finite_vector(const Eigen::VectorXd & values)
{
  return values.size() > 0 && values.allFinite();
}

std::string describe_info(const OSQPInfo * info)
{
  if (info == nullptr) {
    return "info=unavailable";
  }
  std::ostringstream detail;
  detail << "status=" << info->status << ", status_val=" << info->status_val
         << ", iter=" << info->iter << ", pri_res=" << info->pri_res
         << ", dua_res=" << info->dua_res;
  return detail.str();
}

template<typename Vector>
void shift_stage_block(
  const Vector & source, Vector & destination,
  const std::size_t source_offset,
  const std::size_t destination_offset,
  const std::size_t stage_count,
  const std::size_t stage_dimension)
{
  if (stage_count == 0U || stage_dimension == 0U) {
    return;
  }
  for (std::size_t stage = 0U; stage + 1U < stage_count; ++stage) {
    for (std::size_t element = 0U; element < stage_dimension; ++element) {
      destination[static_cast<Eigen::Index>(
          destination_offset + stage * stage_dimension + element)] =
        source[static_cast<Eigen::Index>(
            source_offset + (stage + 1U) * stage_dimension + element)];
    }
  }
  const std::size_t last_stage = stage_count - 1U;
  for (std::size_t element = 0U; element < stage_dimension; ++element) {
    destination[static_cast<Eigen::Index>(
        destination_offset + last_stage * stage_dimension + element)] =
      source[static_cast<Eigen::Index>(
          source_offset + last_stage * stage_dimension + element)];
  }
}

struct PreparedProblem
{
  Eigen::SparseMatrix<double> quadratic_cost;
  Eigen::SparseMatrix<double> constraints;
  std::vector<c_float> quadratic_values;
  std::vector<c_int> quadratic_rows;
  std::vector<c_int> quadratic_columns;
  std::vector<c_float> constraint_values;
  std::vector<c_int> constraint_rows;
  std::vector<c_int> constraint_columns;
  std::vector<c_float> linear_cost;
  std::vector<c_float> lower_bound;
  std::vector<c_float> upper_bound;
};

std::optional<PreparedProblem> prepare_problem(
  Eigen::SparseMatrix<double> quadratic_cost,
  Eigen::SparseMatrix<double> constraints, const Eigen::VectorXd & linear_cost,
  const Eigen::VectorXd & lower_bound, const Eigen::VectorXd & upper_bound,
  std::string & failure_detail)
{
  quadratic_cost.makeCompressed();
  constraints.makeCompressed();
  if (quadratic_cost.rows() <= 0 ||
    quadratic_cost.rows() != quadratic_cost.cols() ||
    linear_cost.size() != quadratic_cost.cols() || constraints.rows() <= 0 ||
    constraints.cols() != quadratic_cost.cols() ||
    constraints.rows() != lower_bound.size() ||
    lower_bound.size() != upper_bound.size() ||
    !sparse_values_are_finite(quadratic_cost) ||
    !sparse_values_are_finite(constraints) || !linear_cost.allFinite())
  {
    failure_detail = "stage=validation, reason=invalid dimensions or "
      "non-finite matrix/vector";
    return std::nullopt;
  }
  for (Eigen::Index index = 0; index < lower_bound.size(); ++index) {
    if (std::isnan(lower_bound[index]) || std::isnan(upper_bound[index]) ||
      lower_bound[index] > upper_bound[index])
    {
      std::ostringstream detail;
      detail << "stage=validation, reason=invalid bounds, index=" << index
             << ", lower=" << lower_bound[index]
             << ", upper=" << upper_bound[index];
      failure_detail = detail.str();
      return std::nullopt;
    }
  }

  PreparedProblem problem;
  problem.quadratic_cost = std::move(quadratic_cost);
  problem.constraints = std::move(constraints);
  problem.quadratic_values.resize(
    static_cast<std::size_t>(problem.quadratic_cost.nonZeros()));
  problem.quadratic_rows.resize(problem.quadratic_values.size());
  problem.quadratic_columns.resize(
    static_cast<std::size_t>(problem.quadratic_cost.cols() + 1));
  for (int index = 0; index < problem.quadratic_cost.nonZeros(); ++index) {
    problem.quadratic_values[static_cast<std::size_t>(index)] =
      static_cast<c_float>(problem.quadratic_cost.valuePtr()[index]);
    problem.quadratic_rows[static_cast<std::size_t>(index)] =
      static_cast<c_int>(problem.quadratic_cost.innerIndexPtr()[index]);
  }
  for (int index = 0; index < problem.quadratic_cost.cols() + 1; ++index) {
    problem.quadratic_columns[static_cast<std::size_t>(index)] =
      static_cast<c_int>(problem.quadratic_cost.outerIndexPtr()[index]);
  }

  problem.constraint_values.resize(
    static_cast<std::size_t>(problem.constraints.nonZeros()));
  problem.constraint_rows.resize(problem.constraint_values.size());
  problem.constraint_columns.resize(
    static_cast<std::size_t>(problem.constraints.cols() + 1));
  for (int index = 0; index < problem.constraints.nonZeros(); ++index) {
    problem.constraint_values[static_cast<std::size_t>(index)] =
      static_cast<c_float>(problem.constraints.valuePtr()[index]);
    problem.constraint_rows[static_cast<std::size_t>(index)] =
      static_cast<c_int>(problem.constraints.innerIndexPtr()[index]);
  }
  for (int index = 0; index < problem.constraints.cols() + 1; ++index) {
    problem.constraint_columns[static_cast<std::size_t>(index)] =
      static_cast<c_int>(problem.constraints.outerIndexPtr()[index]);
  }

  const auto copy_dense = [](const Eigen::VectorXd & source) {
      std::vector<c_float> destination(static_cast<std::size_t>(source.size()));
      for (Eigen::Index index = 0; index < source.size(); ++index) {
        destination[static_cast<std::size_t>(index)] =
          static_cast<c_float>(source[index]);
      }
      return destination;
    };
  problem.linear_cost = copy_dense(linear_cost);
  problem.lower_bound = copy_dense(lower_bound);
  problem.upper_bound = copy_dense(upper_bound);
  return problem;
}

} // namespace

std::optional<WarmStart>
shift_mpc_warm_start(
  const WarmStart & previous, const std::size_t horizon_steps,
  const std::size_t state_dimension,
  const std::size_t input_dimension) noexcept
{
  if (horizon_steps == 0U || state_dimension == 0U || input_dimension == 0U ||
    !finite_vector(previous.primal) || !finite_vector(previous.dual))
  {
    return std::nullopt;
  }
  const std::size_t state_stage_count = horizon_steps + 1U;
  const std::size_t state_values = state_stage_count * state_dimension;
  const std::size_t input_values = horizon_steps * input_dimension;
  const std::size_t primal_size = state_values + input_values;
  const std::size_t dual_size = state_values + primal_size + horizon_steps;
  if (previous.primal.size() != static_cast<Eigen::Index>(primal_size) ||
    previous.dual.size() != static_cast<Eigen::Index>(dual_size))
  {
    return std::nullopt;
  }

  WarmStart shifted{
    Eigen::VectorXd::Zero(static_cast<Eigen::Index>(primal_size)),
    Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dual_size))};
  shift_stage_block(
    previous.primal, shifted.primal, 0U, 0U, state_stage_count,
    state_dimension);
  shift_stage_block(
    previous.primal, shifted.primal, state_values, state_values,
    horizon_steps, input_dimension);

  const std::size_t equality_offset = 0U;
  const std::size_t box_offset = state_values;
  const std::size_t box_input_offset = box_offset + state_values;
  const std::size_t rate_offset = box_offset + primal_size;
  shift_stage_block(
    previous.dual, shifted.dual, equality_offset,
    equality_offset, state_stage_count, state_dimension);
  shift_stage_block(
    previous.dual, shifted.dual, box_offset, box_offset,
    state_stage_count, state_dimension);
  shift_stage_block(
    previous.dual, shifted.dual, box_input_offset,
    box_input_offset, horizon_steps, input_dimension);
  shift_stage_block(
    previous.dual, shifted.dual, rate_offset, rate_offset,
    horizon_steps, 1U);
  if (!shifted.primal.allFinite() || !shifted.dual.allFinite()) {
    return std::nullopt;
  }
  return shifted;
}

struct PersistentOsqpSolver::Impl
{
  std::unique_ptr<OSQPWorkspace, WorkspaceDeleter> workspace;
  std::unique_ptr<csc, CscDeleter> quadratic_csc;
  std::unique_ptr<csc, CscDeleter> constraint_csc;
  OSQPSettings settings{};
  std::vector<c_float> quadratic_values;
  std::vector<c_int> quadratic_rows;
  std::vector<c_int> quadratic_columns;
  std::vector<c_float> constraint_values;
  std::vector<c_int> constraint_rows;
  std::vector<c_int> constraint_columns;
  std::vector<c_float> linear_cost;
  std::vector<c_float> lower_bound;
  std::vector<c_float> upper_bound;
  c_int variable_count{};
  c_int constraint_count{};

  Impl()
  {
    osqp_set_default_settings(&settings);
    settings.verbose = false;
    settings.warm_start = true;
  }

  void reset() noexcept
  {
    workspace.reset();
    quadratic_csc.reset();
    constraint_csc.reset();
    quadratic_values.clear();
    quadratic_rows.clear();
    quadratic_columns.clear();
    constraint_values.clear();
    constraint_rows.clear();
    constraint_columns.clear();
    linear_cost.clear();
    lower_bound.clear();
    upper_bound.clear();
    variable_count = 0;
    constraint_count = 0;
  }

  bool same_structure(const PreparedProblem & problem) const noexcept
  {
    return workspace != nullptr &&
           variable_count == problem.quadratic_cost.cols() &&
           constraint_count == problem.constraints.rows() &&
           quadratic_rows == problem.quadratic_rows &&
           quadratic_columns == problem.quadratic_columns &&
           constraint_rows == problem.constraint_rows &&
           constraint_columns == problem.constraint_columns;
  }

  bool setup(const PreparedProblem & problem, std::string & failure_detail)
  {
    reset();
    variable_count = static_cast<c_int>(problem.quadratic_cost.cols());
    constraint_count = static_cast<c_int>(problem.constraints.rows());
    quadratic_values = problem.quadratic_values;
    quadratic_rows = problem.quadratic_rows;
    quadratic_columns = problem.quadratic_columns;
    constraint_values = problem.constraint_values;
    constraint_rows = problem.constraint_rows;
    constraint_columns = problem.constraint_columns;
    linear_cost = problem.linear_cost;
    lower_bound = problem.lower_bound;
    upper_bound = problem.upper_bound;
    quadratic_csc.reset(
      csc_matrix(
        variable_count, variable_count,
        static_cast<c_int>(quadratic_values.size()), quadratic_values.data(),
        quadratic_rows.data(), quadratic_columns.data()));
    constraint_csc.reset(
      csc_matrix(
        constraint_count, variable_count,
        static_cast<c_int>(constraint_values.size()), constraint_values.data(),
        constraint_rows.data(), constraint_columns.data()));
    if (!quadratic_csc || !constraint_csc) {
      failure_detail = "stage=csc, reason=matrix allocation failed";
      reset();
      return false;
    }
    OSQPData data;
    data.n = variable_count;
    data.m = constraint_count;
    data.P = quadratic_csc.get();
    data.A = constraint_csc.get();
    data.q = linear_cost.data();
    data.l = lower_bound.data();
    data.u = upper_bound.data();
    OSQPWorkspace * raw_workspace = nullptr;
    const c_int exit_flag = osqp_setup(&raw_workspace, &data, &settings);
    workspace.reset(raw_workspace);
    if (exit_flag != 0 || !workspace) {
      std::ostringstream detail;
      detail << "stage=setup, exit_flag=" << exit_flag;
      failure_detail = detail.str();
      reset();
      return false;
    }
    return true;
  }

  bool update(const PreparedProblem & problem, std::string & failure_detail)
  {
    if (!same_structure(problem)) {
      failure_detail = "stage=update, reason=structure mismatch";
      return false;
    }
    std::copy(
      problem.quadratic_values.begin(), problem.quadratic_values.end(),
      quadratic_values.begin());
    std::copy(
      problem.constraint_values.begin(),
      problem.constraint_values.end(), constraint_values.begin());
    std::copy(
      problem.linear_cost.begin(), problem.linear_cost.end(),
      linear_cost.begin());
    std::copy(
      problem.lower_bound.begin(), problem.lower_bound.end(),
      lower_bound.begin());
    std::copy(
      problem.upper_bound.begin(), problem.upper_bound.end(),
      upper_bound.begin());
    const c_int matrix_exit = osqp_update_P_A(
      workspace.get(), quadratic_values.data(), OSQP_NULL,
      static_cast<c_int>(quadratic_values.size()), constraint_values.data(),
      OSQP_NULL, static_cast<c_int>(constraint_values.size()));
    const c_int cost_exit =
      matrix_exit == 0 ?
      osqp_update_lin_cost(workspace.get(), linear_cost.data()) :
      matrix_exit;
    const c_int bounds_exit =
      cost_exit == 0 ? osqp_update_bounds(
      workspace.get(), lower_bound.data(),
      upper_bound.data()) :
      cost_exit;
    if (matrix_exit != 0 || cost_exit != 0 || bounds_exit != 0) {
      std::ostringstream detail;
      detail << "stage=update, matrix_exit=" << matrix_exit
             << ", cost_exit=" << cost_exit << ", bounds_exit=" << bounds_exit;
      failure_detail = detail.str();
      return false;
    }
    return true;
  }
};

PersistentOsqpSolver::PersistentOsqpSolver()
: impl_(std::make_unique<Impl>()) {}

PersistentOsqpSolver::~PersistentOsqpSolver() = default;
PersistentOsqpSolver::PersistentOsqpSolver(PersistentOsqpSolver &&) noexcept =
  default;
PersistentOsqpSolver &
PersistentOsqpSolver::operator=(PersistentOsqpSolver &&) noexcept = default;

void PersistentOsqpSolver::reset() noexcept {impl_->reset();}

bool PersistentOsqpSolver::initialized() const noexcept
{
  return impl_->workspace != nullptr;
}

SolveOutcome PersistentOsqpSolver::solve(
  Eigen::SparseMatrix<double> quadratic_cost,
  Eigen::SparseMatrix<double> constraints, const Eigen::VectorXd & linear_cost,
  const Eigen::VectorXd & lower_bound, const Eigen::VectorXd & upper_bound,
  const std::optional<WarmStart> & warm_start)
{
  SolveOutcome outcome;
  const auto total_start = SteadyClock::now();
  std::string preparation_failure;
  auto prepared = prepare_problem(
    std::move(quadratic_cost), std::move(constraints), linear_cost,
    lower_bound, upper_bound, preparation_failure);
  if (!prepared.has_value()) {
    outcome.failure_detail = std::move(preparation_failure);
    outcome.telemetry.cold_reset_after_failure = impl_->workspace != nullptr;
    impl_->reset();
    outcome.telemetry.total_ms = elapsed_ms(total_start);
    return outcome;
  }

  const bool structure_matches = impl_->same_structure(prepared.value());
  if (!structure_matches) {
    outcome.telemetry.structural_rebuild = impl_->workspace != nullptr;
    const auto setup_start = SteadyClock::now();
    if (!impl_->setup(prepared.value(), outcome.failure_detail)) {
      outcome.telemetry.setup_ms = elapsed_ms(setup_start);
      outcome.telemetry.setup_performed = true;
      outcome.telemetry.total_ms = elapsed_ms(total_start);
      return outcome;
    }
    outcome.telemetry.setup_ms = elapsed_ms(setup_start);
    outcome.telemetry.setup_performed = true;
  } else {
    const auto update_start = SteadyClock::now();
    std::string update_failure;
    if (!impl_->update(prepared.value(), update_failure)) {
      outcome.telemetry.update_ms = elapsed_ms(update_start);
      outcome.telemetry.update_performed = true;
      outcome.telemetry.update_rebuild = true;
      const auto setup_start = SteadyClock::now();
      if (!impl_->setup(prepared.value(), outcome.failure_detail)) {
        outcome.failure_detail = update_failure + "; " + outcome.failure_detail;
        outcome.telemetry.setup_ms = elapsed_ms(setup_start);
        outcome.telemetry.setup_performed = true;
        outcome.telemetry.total_ms = elapsed_ms(total_start);
        return outcome;
      }
      outcome.telemetry.setup_ms = elapsed_ms(setup_start);
      outcome.telemetry.setup_performed = true;
    } else {
      outcome.telemetry.update_ms = elapsed_ms(update_start);
      outcome.telemetry.update_performed = true;
    }
  }

  if (warm_start.has_value()) {
    const bool warm_start_valid =
      finite_vector(warm_start->primal) && finite_vector(warm_start->dual) &&
      warm_start->primal.size() == impl_->variable_count &&
      warm_start->dual.size() == impl_->constraint_count;
    if (warm_start_valid) {
      std::vector<c_float> primal(
        static_cast<std::size_t>(warm_start->primal.size()));
      std::vector<c_float> dual(
        static_cast<std::size_t>(warm_start->dual.size()));
      for (Eigen::Index index = 0; index < warm_start->primal.size(); ++index) {
        primal[static_cast<std::size_t>(index)] =
          static_cast<c_float>(warm_start->primal[index]);
      }
      for (Eigen::Index index = 0; index < warm_start->dual.size(); ++index) {
        dual[static_cast<std::size_t>(index)] =
          static_cast<c_float>(warm_start->dual[index]);
      }
      const auto warm_start_begin = SteadyClock::now();
      const c_int warm_start_exit =
        osqp_warm_start(impl_->workspace.get(), primal.data(), dual.data());
      outcome.telemetry.warm_start_ms = elapsed_ms(warm_start_begin);
      outcome.telemetry.warm_start_applied = warm_start_exit == 0;
      outcome.telemetry.warm_start_rejected = warm_start_exit != 0;
    } else {
      outcome.telemetry.warm_start_rejected = true;
    }
  }

  // OSQP otherwise retains its previous internal iterate when the workspace is
  // numerically updated. If the caller deliberately omitted or rejected a
  // stale warm start, reset that iterate to zero rather than silently reusing
  // the unshifted previous horizon.
  const bool reused_workspace_without_warm_start =
    outcome.telemetry.update_performed &&
    !outcome.telemetry.setup_performed &&
    !outcome.telemetry.warm_start_applied;
  if (reused_workspace_without_warm_start) {
    std::vector<c_float> zero_primal(
      static_cast<std::size_t>(impl_->variable_count), 0.0);
    std::vector<c_float> zero_dual(
      static_cast<std::size_t>(impl_->constraint_count), 0.0);
    const c_int cold_start_exit = osqp_warm_start(
      impl_->workspace.get(), zero_primal.data(), zero_dual.data());
    if (cold_start_exit != 0) {
      outcome.telemetry.update_rebuild = true;
      const auto setup_start = SteadyClock::now();
      std::string setup_failure;
      if (!impl_->setup(prepared.value(), setup_failure)) {
        std::ostringstream detail;
        detail << "stage=cold_start, exit_flag=" << cold_start_exit << "; "
               << setup_failure;
        outcome.failure_detail = detail.str();
        outcome.telemetry.setup_ms += elapsed_ms(setup_start);
        outcome.telemetry.setup_performed = true;
        outcome.telemetry.cold_reset_after_failure = true;
        outcome.telemetry.total_ms = elapsed_ms(total_start);
        return outcome;
      }
      outcome.telemetry.setup_ms += elapsed_ms(setup_start);
      outcome.telemetry.setup_performed = true;
    }
  }

  const auto solve_start = SteadyClock::now();
  const c_int solve_exit = osqp_solve(impl_->workspace.get());
  outcome.telemetry.solve_ms = elapsed_ms(solve_start);
  const OSQPInfo * info = impl_->workspace ? impl_->workspace->info : nullptr;
  outcome.telemetry.iterations = info ? static_cast<int>(info->iter) : 0;
  outcome.telemetry.status = info ? static_cast<int>(info->status_val) : 0;
  if (solve_exit != 0 || info == nullptr) {
    std::ostringstream detail;
    detail << "stage=solve, exit_flag=" << solve_exit << ", "
           << describe_info(info);
    outcome.failure_detail = detail.str();
    impl_->reset();
    outcome.telemetry.cold_reset_after_failure = true;
    outcome.telemetry.total_ms = elapsed_ms(total_start);
    return outcome;
  }
  if (info->status_val != OSQP_SOLVED &&
    info->status_val != OSQP_SOLVED_INACCURATE)
  {
    outcome.failure_detail = "stage=status, " + describe_info(info);
    impl_->reset();
    outcome.telemetry.cold_reset_after_failure = true;
    outcome.telemetry.total_ms = elapsed_ms(total_start);
    return outcome;
  }
  if (impl_->workspace->solution == nullptr ||
    impl_->workspace->solution->x == nullptr ||
    impl_->workspace->solution->y == nullptr)
  {
    outcome.failure_detail =
      "stage=solution, reason=missing primal/dual solution, " +
      describe_info(info);
    impl_->reset();
    outcome.telemetry.cold_reset_after_failure = true;
    outcome.telemetry.total_ms = elapsed_ms(total_start);
    return outcome;
  }

  Eigen::VectorXd primal(impl_->variable_count);
  Eigen::VectorXd dual(impl_->constraint_count);
  for (Eigen::Index index = 0; index < primal.size(); ++index) {
    primal[index] = static_cast<double>(impl_->workspace->solution->x[index]);
  }
  for (Eigen::Index index = 0; index < dual.size(); ++index) {
    dual[index] = static_cast<double>(impl_->workspace->solution->y[index]);
  }
  if (!primal.allFinite() || !dual.allFinite()) {
    outcome.failure_detail =
      "stage=solution, reason=non-finite primal/dual solution, " +
      describe_info(info);
    impl_->reset();
    outcome.telemetry.cold_reset_after_failure = true;
    outcome.telemetry.total_ms = elapsed_ms(total_start);
    return outcome;
  }

  const Eigen::VectorXd constraint_values = prepared->constraints * primal;
  if (constraint_values.size() != lower_bound.size() ||
    !constraint_values.allFinite())
  {
    outcome.failure_detail =
      "stage=constraint_check, reason=invalid projected constraints, " +
      describe_info(info);
    impl_->reset();
    outcome.telemetry.cold_reset_after_failure = true;
    outcome.telemetry.total_ms = elapsed_ms(total_start);
    return outcome;
  }
  double maximum_violation = 0.0;
  double maximum_projected_absolute = 0.0;
  for (Eigen::Index index = 0; index < constraint_values.size(); ++index) {
    double projected = constraint_values[index];
    if (std::isfinite(lower_bound[index])) {
      projected = std::max(projected, lower_bound[index]);
    }
    if (std::isfinite(upper_bound[index])) {
      projected = std::min(projected, upper_bound[index]);
    }
    maximum_violation = std::max(
      maximum_violation, std::abs(constraint_values[index] - projected));
    maximum_projected_absolute =
      std::max(maximum_projected_absolute, std::abs(projected));
  }
  const double constraint_scale = std::max(
    constraint_values.cwiseAbs().maxCoeff(), maximum_projected_absolute);
  const double inaccurate_multiplier =
    info->status_val == OSQP_SOLVED_INACCURATE ? 10.0 : 1.0;
  const double tolerance =
    inaccurate_multiplier *
    (static_cast<double>(impl_->settings.eps_abs) +
    static_cast<double>(impl_->settings.eps_rel) * constraint_scale);
  if (maximum_violation > tolerance) {
    std::ostringstream detail;
    detail << "stage=constraint_check, max_violation=" << maximum_violation
           << ", tolerance=" << tolerance << ", " << describe_info(info);
    outcome.failure_detail = detail.str();
    impl_->reset();
    outcome.telemetry.cold_reset_after_failure = true;
    outcome.telemetry.total_ms = elapsed_ms(total_start);
    return outcome;
  }

  outcome.result =
    SolveResult{std::move(primal), std::move(dual),
    static_cast<int>(info->status_val), maximum_violation};
  outcome.telemetry.total_ms = elapsed_ms(total_start);
  return outcome;
}

} // namespace multi_purpose_mpc_ros::persistent_osqp
