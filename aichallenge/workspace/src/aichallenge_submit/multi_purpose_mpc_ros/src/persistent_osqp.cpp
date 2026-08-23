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
  /// Original physical quadratic cost used for dimensions/provenance.
  Eigen::SparseMatrix<double> quadratic_cost;
  /// Numerically transformed quadratic cost passed to OSQP.
  Eigen::SparseMatrix<double> solver_quadratic_cost;
  /// Original physical constraint matrix used for the returned certificate.
  Eigen::SparseMatrix<double> constraints;
  /// Numerically preconditioned matrix passed to OSQP.
  Eigen::SparseMatrix<double> solver_constraints;
  /// S_ii in solver_constraints = S * constraints.
  Eigen::VectorXd constraint_row_scale;
  /// D_jj in physical_primal = D * solver_primal.
  Eigen::VectorXd variable_scale;
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
  const ConstraintPreconditioningPolicy preconditioning_policy,
  const double absolute_tolerance, const double relative_tolerance,
  const std::optional<VariableCoordinateScaling> & variable_scaling,
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
  problem.solver_quadratic_cost = problem.quadratic_cost;
  problem.constraints = std::move(constraints);
  problem.solver_constraints = problem.constraints;
  problem.constraint_row_scale =
    Eigen::VectorXd::Ones(problem.constraints.rows());
  problem.variable_scale = Eigen::VectorXd::Ones(problem.constraints.cols());
  if (variable_scaling.has_value()) {
    const auto & scale = variable_scaling->physical_units_per_solver_unit;
    if (
      scale.size() != problem.constraints.cols() || !scale.allFinite() ||
      (scale.array() <= 0.0).any())
    {
      failure_detail =
        "stage=preconditioning, reason=invalid variable-coordinate scale";
      return std::nullopt;
    }
    problem.variable_scale = scale;
    for (
      int column = 0; column < problem.solver_quadratic_cost.outerSize();
      ++column)
    {
      for (
        Eigen::SparseMatrix<double>::InnerIterator entry(
          problem.solver_quadratic_cost, column);
        entry; ++entry)
      {
        entry.valueRef() *=
          problem.variable_scale[entry.row()] *
          problem.variable_scale[entry.col()];
      }
    }
    for (
      int column = 0; column < problem.solver_constraints.outerSize();
      ++column)
    {
      for (
        Eigen::SparseMatrix<double>::InnerIterator entry(
          problem.solver_constraints, column);
        entry; ++entry)
      {
        entry.valueRef() *= problem.variable_scale[entry.col()];
      }
    }
    if (
      !sparse_values_are_finite(problem.solver_quadratic_cost) ||
      !sparse_values_are_finite(problem.solver_constraints))
    {
      failure_detail =
        "stage=preconditioning, reason=non-finite variable-scaled matrix";
      return std::nullopt;
    }
  }
  if (preconditioning_policy ==
    ConstraintPreconditioningPolicy::RowToleranceNormalized)
  {
    if (!std::isfinite(absolute_tolerance) || absolute_tolerance < 0.0 ||
      !std::isfinite(relative_tolerance) || relative_tolerance < 0.0)
    {
      failure_detail =
        "stage=preconditioning, reason=invalid physical tolerances";
      return std::nullopt;
    }
    Eigen::VectorXd row_tolerance =
      Eigen::VectorXd::Zero(problem.constraints.rows());
    for (Eigen::Index row = 0; row < lower_bound.size(); ++row) {
      const bool finite_lower = std::isfinite(lower_bound[row]);
      const bool finite_upper = std::isfinite(upper_bound[row]);
      if (!finite_lower && !finite_upper) {
        continue;
      }
      double tolerance = std::numeric_limits<double>::infinity();
      if (finite_lower) {
        tolerance = std::min(
          tolerance,
          absolute_tolerance + relative_tolerance * std::abs(lower_bound[row]));
      }
      if (finite_upper) {
        tolerance = std::min(
          tolerance,
          absolute_tolerance + relative_tolerance * std::abs(upper_bound[row]));
      }
      if (!std::isfinite(tolerance) || tolerance <= 0.0) {
        std::ostringstream detail;
        detail << "stage=preconditioning, reason=invalid row tolerance, row="
               << row << ", tolerance=" << tolerance;
        failure_detail = detail.str();
        return std::nullopt;
      }
      row_tolerance[row] = tolerance;
    }
    for (Eigen::Index row = 0; row < row_tolerance.size(); ++row) {
      if (row_tolerance[row] == 0.0) {
        continue;
      }
      // Map the strictest physical side tolerance to OSQP's absolute stopping
      // tolerance. The normalized solver disables its global relative term,
      // because that relative term has already been included in
      // row_tolerance and must not be applied again using another row's scale.
      const double row_scale = absolute_tolerance / row_tolerance[row];
      if (!std::isfinite(row_scale) || row_scale <= 0.0) {
        std::ostringstream detail;
        detail << "stage=preconditioning, reason=invalid row scale, row="
               << row << ", scale=" << row_scale;
        failure_detail = detail.str();
        return std::nullopt;
      }
      problem.constraint_row_scale[row] = row_scale;
    }
    for (
      int column = 0; column < problem.solver_constraints.outerSize();
      ++column)
    {
      for (
        Eigen::SparseMatrix<double>::InnerIterator entry(
          problem.solver_constraints, column);
        entry; ++entry)
      {
        entry.valueRef() *= problem.constraint_row_scale[entry.row()];
      }
    }
    if (!sparse_values_are_finite(problem.solver_constraints)) {
      failure_detail =
        "stage=preconditioning, reason=non-finite scaled constraint matrix";
      return std::nullopt;
    }
  }
  problem.quadratic_values.resize(
    static_cast<std::size_t>(problem.solver_quadratic_cost.nonZeros()));
  problem.quadratic_rows.resize(problem.quadratic_values.size());
  problem.quadratic_columns.resize(
    static_cast<std::size_t>(problem.quadratic_cost.cols() + 1));
  for (int index = 0; index < problem.solver_quadratic_cost.nonZeros(); ++index) {
    problem.quadratic_values[static_cast<std::size_t>(index)] =
      static_cast<c_float>(problem.solver_quadratic_cost.valuePtr()[index]);
    problem.quadratic_rows[static_cast<std::size_t>(index)] =
      static_cast<c_int>(problem.solver_quadratic_cost.innerIndexPtr()[index]);
  }
  for (int index = 0; index < problem.solver_quadratic_cost.cols() + 1; ++index) {
    problem.quadratic_columns[static_cast<std::size_t>(index)] =
      static_cast<c_int>(problem.solver_quadratic_cost.outerIndexPtr()[index]);
  }

  problem.constraint_values.resize(
    static_cast<std::size_t>(problem.solver_constraints.nonZeros()));
  problem.constraint_rows.resize(problem.constraint_values.size());
  problem.constraint_columns.resize(
    static_cast<std::size_t>(problem.solver_constraints.cols() + 1));
  for (int index = 0; index < problem.solver_constraints.nonZeros(); ++index) {
    problem.constraint_values[static_cast<std::size_t>(index)] =
      static_cast<c_float>(problem.solver_constraints.valuePtr()[index]);
    problem.constraint_rows[static_cast<std::size_t>(index)] =
      static_cast<c_int>(problem.solver_constraints.innerIndexPtr()[index]);
  }
  for (int index = 0; index < problem.solver_constraints.cols() + 1; ++index) {
    problem.constraint_columns[static_cast<std::size_t>(index)] =
      static_cast<c_int>(problem.solver_constraints.outerIndexPtr()[index]);
  }

  const auto copy_dense = [](const Eigen::VectorXd & source) {
      std::vector<c_float> destination(static_cast<std::size_t>(source.size()));
      for (Eigen::Index index = 0; index < source.size(); ++index) {
        destination[static_cast<std::size_t>(index)] =
          static_cast<c_float>(source[index]);
      }
      return destination;
    };
  problem.linear_cost = copy_dense(
    linear_cost.cwiseProduct(problem.variable_scale));
  Eigen::VectorXd solver_lower_bound = lower_bound;
  Eigen::VectorXd solver_upper_bound = upper_bound;
  for (Eigen::Index row = 0; row < lower_bound.size(); ++row) {
    if (std::isfinite(solver_lower_bound[row])) {
      solver_lower_bound[row] *= problem.constraint_row_scale[row];
      if (!std::isfinite(solver_lower_bound[row])) {
        failure_detail =
          "stage=preconditioning, reason=non-finite scaled lower bound";
        return std::nullopt;
      }
    }
    if (std::isfinite(solver_upper_bound[row])) {
      solver_upper_bound[row] *= problem.constraint_row_scale[row];
      if (!std::isfinite(solver_upper_bound[row])) {
        failure_detail =
          "stage=preconditioning, reason=non-finite scaled upper bound";
        return std::nullopt;
      }
    }
  }
  problem.lower_bound = copy_dense(solver_lower_bound);
  problem.upper_bound = copy_dense(solver_upper_bound);
  return problem;
}

} // namespace

std::optional<VariableCoordinateScaling>
derive_box_variable_coordinate_scaling(
  const Eigen::VectorXd & lower_bound,
  const Eigen::VectorXd & upper_bound) noexcept
{
  if (
    lower_bound.size() <= 0 || lower_bound.size() != upper_bound.size())
  {
    return std::nullopt;
  }
  Eigen::VectorXd scale = Eigen::VectorXd::Ones(lower_bound.size());
  for (Eigen::Index index = 0; index < lower_bound.size(); ++index) {
    if (
      std::isnan(lower_bound[index]) || std::isnan(upper_bound[index]) ||
      lower_bound[index] > upper_bound[index])
    {
      return std::nullopt;
    }
    double characteristic = 0.0;
    if (std::isfinite(lower_bound[index])) {
      characteristic = std::max(characteristic, std::abs(lower_bound[index]));
    }
    if (std::isfinite(upper_bound[index])) {
      characteristic = std::max(characteristic, std::abs(upper_bound[index]));
    }
    if (characteristic > 0.0) {
      scale[index] = characteristic;
    }
  }
  if (!scale.allFinite() || (scale.array() <= 0.0).any()) {
    return std::nullopt;
  }
  return VariableCoordinateScaling{std::move(scale)};
}

std::optional<ConstraintResidualReport> evaluate_constraint_residuals(
  const Eigen::SparseMatrix<double> & constraints,
  const Eigen::VectorXd & primal,
  const Eigen::VectorXd & lower_bound,
  const Eigen::VectorXd & upper_bound,
  const double absolute_tolerance,
  const double relative_tolerance,
  const double tolerance_multiplier) noexcept
{
  if (
    constraints.rows() <= 0 || constraints.cols() != primal.size() ||
    constraints.rows() != lower_bound.size() ||
    lower_bound.size() != upper_bound.size() || !primal.allFinite() ||
    !std::isfinite(absolute_tolerance) || absolute_tolerance < 0.0 ||
    !std::isfinite(relative_tolerance) || relative_tolerance < 0.0 ||
    !std::isfinite(tolerance_multiplier) || tolerance_multiplier <= 0.0)
  {
    return std::nullopt;
  }
  const Eigen::VectorXd values = constraints * primal;
  if (!values.allFinite()) {
    return std::nullopt;
  }

  ConstraintResidualReport report;
  report.violation = Eigen::VectorXd::Zero(values.size());
  report.tolerance = Eigen::VectorXd::Zero(values.size());
  for (Eigen::Index row = 0; row < values.size(); ++row) {
    if (
      std::isnan(lower_bound[row]) || std::isnan(upper_bound[row]) ||
      lower_bound[row] > upper_bound[row])
    {
      return std::nullopt;
    }
    double projected = values[row];
    if (std::isfinite(lower_bound[row])) {
      projected = std::max(projected, lower_bound[row]);
    }
    if (std::isfinite(upper_bound[row])) {
      projected = std::min(projected, upper_bound[row]);
    }
    const double violation = std::abs(values[row] - projected);
    const double scale = std::max(std::abs(values[row]), std::abs(projected));
    const double tolerance = tolerance_multiplier *
      (absolute_tolerance + relative_tolerance * scale);
    if (!std::isfinite(violation) || !std::isfinite(tolerance)) {
      return std::nullopt;
    }
    const double normalized = tolerance > 0.0 ?
      violation / tolerance :
      (violation == 0.0 ? 0.0 : std::numeric_limits<double>::infinity());
    report.violation[row] = violation;
    report.tolerance[row] = tolerance;
    report.maximum_absolute_violation = std::max(
      report.maximum_absolute_violation, violation);
    if (normalized > report.maximum_normalized_violation) {
      report.maximum_normalized_violation = normalized;
      report.maximum_normalized_row = static_cast<int>(row);
    }
  }
  return report;
}

std::optional<WarmStart>
shift_mpc_warm_start(
  const WarmStart & previous, const std::size_t horizon_steps,
  const std::size_t state_dimension,
  const std::size_t input_dimension,
  const std::vector<DualStageBlockLayout> & trailing_dual_stage_blocks) noexcept
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
  const std::size_t base_dual_size = state_values + primal_size + horizon_steps;
  std::size_t trailing_dual_size = 0U;
  for (const auto & block : trailing_dual_stage_blocks) {
    if (
      block.stage_count == 0U || block.rows_per_stage == 0U ||
      block.rows_per_stage >
      std::numeric_limits<std::size_t>::max() / block.stage_count)
    {
      return std::nullopt;
    }
    const std::size_t block_size = block.stage_count * block.rows_per_stage;
    if (
      trailing_dual_size >
      std::numeric_limits<std::size_t>::max() - block_size)
    {
      return std::nullopt;
    }
    trailing_dual_size += block_size;
  }
  if (
    base_dual_size >
    std::numeric_limits<std::size_t>::max() - trailing_dual_size)
  {
    return std::nullopt;
  }
  const std::size_t dual_size = base_dual_size + trailing_dual_size;
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
  std::size_t trailing_offset = base_dual_size;
  for (const auto & block : trailing_dual_stage_blocks) {
    shift_stage_block(
      previous.dual, shifted.dual, trailing_offset, trailing_offset,
      block.stage_count, block.rows_per_stage);
    trailing_offset += block.stage_count * block.rows_per_stage;
  }
  if (!shifted.primal.allFinite() || !shifted.dual.allFinite()) {
    return std::nullopt;
  }
  return shifted;
}

bool CertifiedWarmStartStore::publish(
  WarmStart value, const double certified_sec,
  const double progress_origin_m) noexcept
{
  if (
    !finite_vector(value.primal) || !finite_vector(value.dual) ||
    !std::isfinite(certified_sec) || !std::isfinite(progress_origin_m))
  {
    return false;
  }
  artifact_ = CertifiedWarmStart{
    std::move(value), certified_sec, progress_origin_m};
  return true;
}

std::optional<CertifiedWarmStart> CertifiedWarmStartStore::consume_fresh(
  const double now_sec, const double maximum_age_sec) noexcept
{
  auto consumed = std::move(artifact_);
  artifact_.reset();
  if (
    !consumed.has_value() || !std::isfinite(now_sec) ||
    !std::isfinite(maximum_age_sec) || maximum_age_sec < 0.0 ||
    now_sec < consumed->certified_sec ||
    now_sec - consumed->certified_sec > maximum_age_sec)
  {
    return std::nullopt;
  }
  return consumed;
}

void CertifiedWarmStartStore::reset() noexcept {artifact_.reset();}

bool CertifiedWarmStartStore::available() const noexcept
{
  return artifact_.has_value();
}

std::optional<ConstraintFailureDiagnostic> make_constraint_failure_diagnostic(
  const ConstraintResidualReport & report,
  const Eigen::VectorXd & constraint_values,
  const Eigen::VectorXd & lower_bound,
  const Eigen::VectorXd & upper_bound) noexcept
{
  const int row = report.maximum_normalized_row;
  if (
    row < 0 || row >= constraint_values.size() ||
    constraint_values.size() != lower_bound.size() ||
    lower_bound.size() != upper_bound.size() ||
    report.violation.size() != constraint_values.size() ||
    report.tolerance.size() != constraint_values.size() ||
    !std::isfinite(constraint_values[row]) ||
    !std::isfinite(report.violation[row]) ||
    !std::isfinite(report.tolerance[row]) ||
    !std::isfinite(report.maximum_normalized_violation))
  {
    return std::nullopt;
  }
  double projected = constraint_values[row];
  if (std::isfinite(lower_bound[row])) {
    projected = std::max(projected, lower_bound[row]);
  }
  if (std::isfinite(upper_bound[row])) {
    projected = std::min(projected, upper_bound[row]);
  }
  if (!std::isfinite(projected)) {
    return std::nullopt;
  }
  return ConstraintFailureDiagnostic{
    row,
    constraint_values[row],
    projected,
    lower_bound[row],
    upper_bound[row],
    report.violation[row],
    report.tolerance[row],
    report.maximum_normalized_violation};
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
  ConstraintPreconditioningPolicy preconditioning_policy{
    ConstraintPreconditioningPolicy::None};
  c_float physical_absolute_tolerance{};
  c_float physical_relative_tolerance{};

  explicit Impl(const ConstraintPreconditioningPolicy policy)
  : preconditioning_policy(policy)
  {
    osqp_set_default_settings(&settings);
    physical_absolute_tolerance = settings.eps_abs;
    physical_relative_tolerance = settings.eps_rel;
    // RowToleranceNormalized embeds the physical relative tolerance into each
    // row scale. Leaving OSQP's global relative stopping term enabled would
    // apply relative tolerance a second time using the largest transformed
    // row, recreating the mixed-unit leak the policy is meant to remove.
    if (preconditioning_policy ==
      ConstraintPreconditioningPolicy::RowToleranceNormalized)
    {
      settings.eps_rel = 0.0;
    }
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
: PersistentOsqpSolver(ConstraintPreconditioningPolicy::None) {}

PersistentOsqpSolver::PersistentOsqpSolver(
  const ConstraintPreconditioningPolicy policy)
: impl_(std::make_unique<Impl>(policy)) {}

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
  const std::optional<WarmStart> & warm_start,
  const std::optional<VariableCoordinateScaling> & variable_scaling)
{
  SolveOutcome outcome;
  const auto total_start = SteadyClock::now();
  outcome.telemetry.absolute_tolerance =
    static_cast<double>(impl_->settings.eps_abs);
  outcome.telemetry.relative_tolerance =
    static_cast<double>(impl_->settings.eps_rel);
  outcome.telemetry.scaling_iterations =
    static_cast<int>(impl_->settings.scaling);
  outcome.telemetry.scaled_termination =
    impl_->settings.scaled_termination != 0;
  outcome.telemetry.row_tolerance_preconditioned =
    impl_->preconditioning_policy ==
    ConstraintPreconditioningPolicy::RowToleranceNormalized;
  std::string preparation_failure;
  auto prepared = prepare_problem(
    std::move(quadratic_cost), std::move(constraints), linear_cost,
    lower_bound, upper_bound, impl_->preconditioning_policy,
    static_cast<double>(impl_->physical_absolute_tolerance),
    static_cast<double>(impl_->physical_relative_tolerance), variable_scaling,
    preparation_failure);
  if (!prepared.has_value()) {
    outcome.failure_detail = std::move(preparation_failure);
    outcome.telemetry.cold_reset_after_failure = impl_->workspace != nullptr;
    impl_->reset();
    outcome.telemetry.total_ms = elapsed_ms(total_start);
    return outcome;
  }
  if (prepared->constraint_row_scale.size() > 0) {
    outcome.telemetry.maximum_row_scale =
      prepared->constraint_row_scale.maxCoeff();
  }
  if (prepared->variable_scale.size() > 0) {
    outcome.telemetry.minimum_variable_scale =
      prepared->variable_scale.minCoeff();
    outcome.telemetry.maximum_variable_scale =
      prepared->variable_scale.maxCoeff();
    outcome.telemetry.variable_coordinate_scaled =
      (prepared->variable_scale.array() != 1.0).any();
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
          static_cast<c_float>(
          warm_start->primal[index] / prepared->variable_scale[index]);
      }
      for (Eigen::Index index = 0; index < warm_start->dual.size(); ++index) {
        const double scaled_dual = warm_start->dual[index] /
          prepared->constraint_row_scale[index];
        dual[static_cast<std::size_t>(index)] =
          static_cast<c_float>(scaled_dual);
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
  if (info != nullptr) {
    outcome.telemetry.objective_value = static_cast<double>(info->obj_val);
    outcome.telemetry.primal_residual = static_cast<double>(info->pri_res);
    outcome.telemetry.dual_residual = static_cast<double>(info->dua_res);
    outcome.telemetry.rho_updates = static_cast<int>(info->rho_updates);
    outcome.telemetry.rho_estimate = static_cast<double>(info->rho_estimate);
  }
  outcome.telemetry.maximum_iterations_reached =
    info != nullptr && info->status_val == OSQP_MAX_ITER_REACHED;
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

  Eigen::VectorXd solver_primal(impl_->variable_count);
  Eigen::VectorXd primal(impl_->variable_count);
  Eigen::VectorXd dual(impl_->constraint_count);
  for (Eigen::Index index = 0; index < primal.size(); ++index) {
    solver_primal[index] =
      static_cast<double>(impl_->workspace->solution->x[index]);
    primal[index] = prepared->variable_scale[index] * solver_primal[index];
  }
  for (Eigen::Index index = 0; index < dual.size(); ++index) {
    dual[index] =
      prepared->constraint_row_scale[index] *
      static_cast<double>(impl_->workspace->solution->y[index]);
  }
  if (!solver_primal.allFinite() || !primal.allFinite() || !dual.allFinite()) {
    outcome.failure_detail =
      "stage=solution, reason=non-finite primal/dual solution, " +
      describe_info(info);
    impl_->reset();
    outcome.telemetry.cold_reset_after_failure = true;
    outcome.telemetry.total_ms = elapsed_ms(total_start);
    return outcome;
  }

  const double inaccurate_multiplier =
    info->status_val == OSQP_SOLVED_INACCURATE ? 10.0 : 1.0;
  const auto residual_report = evaluate_constraint_residuals(
    prepared->constraints, primal, lower_bound, upper_bound,
    static_cast<double>(impl_->physical_absolute_tolerance),
    static_cast<double>(impl_->physical_relative_tolerance),
    inaccurate_multiplier);
  if (!residual_report.has_value()) {
    outcome.failure_detail =
      "stage=constraint_check, reason=invalid projected constraints, " +
      describe_info(info);
    impl_->reset();
    outcome.telemetry.cold_reset_after_failure = true;
    outcome.telemetry.total_ms = elapsed_ms(total_start);
    return outcome;
  }
  const Eigen::VectorXd constraint_values = prepared->constraints * primal;
  double maximum_projected_absolute = 0.0;
  for (Eigen::Index index = 0; index < constraint_values.size(); ++index) {
    double projected = constraint_values[index];
    if (std::isfinite(lower_bound[index])) {
      projected = std::max(projected, lower_bound[index]);
    }
    if (std::isfinite(upper_bound[index])) {
      projected = std::min(projected, upper_bound[index]);
    }
    maximum_projected_absolute =
      std::max(maximum_projected_absolute, std::abs(projected));
  }
  const double constraint_scale = std::max(
    constraint_values.cwiseAbs().maxCoeff(), maximum_projected_absolute);
  const double tolerance =
    inaccurate_multiplier *
    (static_cast<double>(impl_->physical_absolute_tolerance) +
    static_cast<double>(impl_->physical_relative_tolerance) * constraint_scale);
  outcome.telemetry.physical_constraint_scale = constraint_scale;
  outcome.telemetry.physical_global_tolerance = tolerance;
  const bool constraint_rejected =
    impl_->preconditioning_policy ==
      ConstraintPreconditioningPolicy::RowToleranceNormalized ?
    residual_report->maximum_normalized_violation > 1.0 :
    residual_report->maximum_absolute_violation > tolerance;
  if (constraint_rejected) {
    const Eigen::VectorXd solver_constraint_values =
      prepared->solver_constraints * solver_primal;
    double solver_exact_primal_residual = 0.0;
    if (solver_constraint_values.allFinite()) {
      for (Eigen::Index row = 0; row < solver_constraint_values.size(); ++row) {
        double violation = 0.0;
        const double solver_lower =
          static_cast<double>(prepared->lower_bound[static_cast<std::size_t>(row)]);
        const double solver_upper =
          static_cast<double>(prepared->upper_bound[static_cast<std::size_t>(row)]);
        if (
          std::isfinite(solver_lower) &&
          solver_constraint_values[row] < solver_lower)
        {
          violation = solver_lower - solver_constraint_values[row];
        }
        if (
          std::isfinite(solver_upper) &&
          solver_constraint_values[row] > solver_upper)
        {
          violation = std::max(
            violation, solver_constraint_values[row] - solver_upper);
        }
        solver_exact_primal_residual =
          std::max(solver_exact_primal_residual, violation);
      }
    } else {
      solver_exact_primal_residual =
        std::numeric_limits<double>::quiet_NaN();
    }
    outcome.constraint_failure = make_constraint_failure_diagnostic(
      residual_report.value(), constraint_values, lower_bound, upper_bound);
    const Eigen::Index rejected_row =
      residual_report->maximum_normalized_row;
    const bool rejected_row_available =
      rejected_row >= 0 && rejected_row < solver_constraint_values.size();
    std::ostringstream detail;
    detail << "stage=constraint_check, max_violation="
           << residual_report->maximum_absolute_violation
           << ", tolerance=" << tolerance
           << ", max_normalized="
           << residual_report->maximum_normalized_violation
           << ", row=" << residual_report->maximum_normalized_row
           << ", solver_exact_pri_res=" << solver_exact_primal_residual
           << ", rejected_row_scale=" <<
      (rejected_row_available ?
      prepared->constraint_row_scale[rejected_row] :
      std::numeric_limits<double>::quiet_NaN())
           << ", rejected_solver_value=" <<
      (rejected_row_available ? solver_constraint_values[rejected_row] :
      std::numeric_limits<double>::quiet_NaN())
           << ", rejected_solver_bounds=[" <<
      (rejected_row_available ?
      static_cast<double>(prepared->lower_bound[
        static_cast<std::size_t>(rejected_row)]) :
      std::numeric_limits<double>::quiet_NaN()) << "," <<
      (rejected_row_available ?
      static_cast<double>(prepared->upper_bound[
        static_cast<std::size_t>(rejected_row)]) :
      std::numeric_limits<double>::quiet_NaN()) << "]"
           << ", " << describe_info(info);
    outcome.failure_detail = detail.str();
    impl_->reset();
    outcome.telemetry.cold_reset_after_failure = true;
    outcome.telemetry.total_ms = elapsed_ms(total_start);
    return outcome;
  }

  outcome.result =
    SolveResult{std::move(primal), std::move(dual),
    static_cast<int>(info->status_val),
    residual_report->maximum_absolute_violation,
    residual_report->violation, residual_report->tolerance,
    residual_report->maximum_normalized_violation,
    residual_report->maximum_normalized_row};
  outcome.telemetry.total_ms = elapsed_ms(total_start);
  return outcome;
}

} // namespace multi_purpose_mpc_ros::persistent_osqp
