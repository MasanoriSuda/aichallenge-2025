#include "multi_purpose_mpc_ros/mpcc_rate_resolved_problem.hpp"

#include <cmath>
#include <limits>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_problem
{
namespace
{

bool finite_non_negative(const Eigen::VectorXd & values) noexcept
{
  return values.allFinite() && (values.array() >= 0.0).all();
}

bool valid_bounds(
  const Eigen::VectorXd & lower, const Eigen::VectorXd & upper) noexcept
{
  if (lower.size() != upper.size()) {
    return false;
  }
  for (Eigen::Index index = 0; index < lower.size(); ++index) {
    if (
      std::isnan(lower[index]) || std::isnan(upper[index]) ||
      lower[index] > upper[index])
    {
      return false;
    }
  }
  return true;
}

}  // namespace

std::optional<Problem> assemble(const AssemblyRequest & request) noexcept
{
  constexpr int nx = model::kStateDimension;
  constexpr int nu = model::kInputDimension;
  const int horizon = request.horizon_steps;
  if (horizon <= 0 || !request.initial_state.allFinite()) {
    return std::nullopt;
  }
  const int state_values = nx * (horizon + 1);
  const int input_values = nu * horizon;
  const int variable_count = state_values + input_values;
  if (
    request.linearizations.size() != static_cast<std::size_t>(horizon) ||
    request.state_reference.size() != state_values ||
    request.state_lower.size() != state_values ||
    request.state_upper.size() != state_values ||
    request.state_weight.size() != state_values ||
    request.input_reference.size() != input_values ||
    request.input_lower.size() != input_values ||
    request.input_upper.size() != input_values ||
    request.input_weight.size() != input_values ||
    !request.state_reference.allFinite() ||
    !request.input_reference.allFinite() ||
    !finite_non_negative(request.state_weight) ||
    !finite_non_negative(request.input_weight) ||
    !request.previous_input.allFinite() ||
    !request.input_delta_weight.allFinite() ||
    (request.input_delta_weight.array() < 0.0).any() ||
    !valid_bounds(request.state_lower, request.state_upper) ||
    !valid_bounds(request.input_lower, request.input_upper))
  {
    return std::nullopt;
  }
  for (const auto & linearization : request.linearizations) {
    if (
      !linearization.state_matrix.allFinite() ||
      !linearization.input_matrix.allFinite() ||
      !linearization.equality_offset.allFinite() ||
      !std::isfinite(linearization.stage_dt_sec) ||
      linearization.stage_dt_sec <= 0.0)
    {
      return std::nullopt;
    }
  }

  for (int element = 0; element < nx; ++element) {
    if (
      request.initial_state[element] < request.state_lower[element] ||
      request.initial_state[element] > request.state_upper[element])
    {
      return std::nullopt;
    }
  }

  std::vector<Eigen::Triplet<double>> constraint_triplets;
  constraint_triplets.reserve(static_cast<std::size_t>(
    state_values + horizon * (nx * nx + nx * nu) + variable_count));
  for (int row = 0; row < state_values; ++row) {
    constraint_triplets.emplace_back(row, row, -1.0);
  }
  for (int stage = 0; stage < horizon; ++stage) {
    const auto & linearization =
      request.linearizations[static_cast<std::size_t>(stage)];
    for (int row = 0; row < nx; ++row) {
      for (int column = 0; column < nx; ++column) {
        const double value = linearization.state_matrix(row, column);
        if (value != 0.0) {
          constraint_triplets.emplace_back(
            (stage + 1) * nx + row, stage * nx + column, value);
        }
      }
      for (int column = 0; column < nu; ++column) {
        const double value = linearization.input_matrix(row, column);
        if (value != 0.0) {
          constraint_triplets.emplace_back(
            (stage + 1) * nx + row,
            state_values + stage * nu + column, value);
        }
      }
    }
  }
  const int box_offset = state_values;
  for (int variable = 0; variable < variable_count; ++variable) {
    constraint_triplets.emplace_back(
      box_offset + variable, variable, 1.0);
  }
  Eigen::SparseMatrix<double> constraints(
    state_values + variable_count, variable_count);
  constraints.setFromTriplets(
    constraint_triplets.begin(), constraint_triplets.end());
  constraints.makeCompressed();

  Eigen::VectorXd equality = Eigen::VectorXd::Zero(state_values);
  equality.segment<nx>(0) = -request.initial_state;
  for (int stage = 0; stage < horizon; ++stage) {
    equality.segment<nx>((stage + 1) * nx) =
      request.linearizations[static_cast<std::size_t>(stage)].equality_offset;
  }
  Eigen::VectorXd box_lower(variable_count);
  Eigen::VectorXd box_upper(variable_count);
  box_lower << request.state_lower, request.input_lower;
  box_upper << request.state_upper, request.input_upper;
  Eigen::VectorXd lower_bound(state_values + variable_count);
  Eigen::VectorXd upper_bound(state_values + variable_count);
  lower_bound << equality, box_lower;
  upper_bound << equality, box_upper;

  Eigen::VectorXd linear_cost = Eigen::VectorXd::Zero(variable_count);
  std::vector<Eigen::Triplet<double>> cost_triplets;
  cost_triplets.reserve(static_cast<std::size_t>(
    variable_count + nu * (3 * horizon - 2)));
  for (int variable = 0; variable < state_values; ++variable) {
    const double weight = request.state_weight[variable];
    if (weight > 0.0) {
      cost_triplets.emplace_back(variable, variable, weight);
      linear_cost[variable] -= weight * request.state_reference[variable];
    }
  }
  for (int input = 0; input < input_values; ++input) {
    const int variable = state_values + input;
    const double weight = request.input_weight[input];
    if (weight > 0.0) {
      cost_triplets.emplace_back(variable, variable, weight);
      linear_cost[variable] -= weight * request.input_reference[input];
    }
  }
  for (int element = 0; element < nu; ++element) {
    const double weight = request.input_delta_weight[element];
    if (weight <= 0.0) {
      continue;
    }
    const int first = state_values + element;
    cost_triplets.emplace_back(first, first, weight);
    linear_cost[first] -= weight * request.previous_input[element];
    for (int stage = 1; stage < horizon; ++stage) {
      const int previous = state_values + (stage - 1) * nu + element;
      const int current = state_values + stage * nu + element;
      cost_triplets.emplace_back(previous, previous, weight);
      cost_triplets.emplace_back(current, current, weight);
      cost_triplets.emplace_back(previous, current, -weight);
    }
  }
  Eigen::SparseMatrix<double> quadratic_cost(variable_count, variable_count);
  quadratic_cost.setFromTriplets(cost_triplets.begin(), cost_triplets.end());
  quadratic_cost.makeCompressed();
  if (!linear_cost.allFinite()) {
    return std::nullopt;
  }
  const auto scaling =
    persistent_osqp::derive_box_variable_coordinate_scaling(
    box_lower, box_upper);
  if (!scaling.has_value()) {
    return std::nullopt;
  }
  return Problem{
    std::move(linear_cost), std::move(lower_bound), std::move(upper_bound),
    std::move(quadratic_cost), std::move(constraints),
    std::move(scaling.value()), horizon};
}

RowSemantic decode_row(const int row, const int horizon_steps) noexcept
{
  constexpr int nx = model::kStateDimension;
  constexpr int nu = model::kInputDimension;
  if (row < 0 || horizon_steps <= 0) {
    return {};
  }
  const int state_values = nx * (horizon_steps + 1);
  const int input_values = nu * horizon_steps;
  if (row < state_values) {
    return RowSemantic{
      true, RowKind::DynamicsEquality, row / nx, row % nx};
  }
  const int box_row = row - state_values;
  if (box_row < state_values) {
    return RowSemantic{
      true, RowKind::StateBox, box_row / nx, box_row % nx};
  }
  const int input_row = box_row - state_values;
  if (input_row < input_values) {
    return RowSemantic{
      true, RowKind::InputBox, input_row / nu, input_row % nu};
  }
  return {};
}

const char * row_kind_name(const RowKind kind) noexcept
{
  switch (kind) {
    case RowKind::Invalid: return "invalid";
    case RowKind::DynamicsEquality: return "dynamics-equality";
    case RowKind::StateBox: return "state-box";
    case RowKind::InputBox: return "input-box";
  }
  return "unknown";
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_problem
