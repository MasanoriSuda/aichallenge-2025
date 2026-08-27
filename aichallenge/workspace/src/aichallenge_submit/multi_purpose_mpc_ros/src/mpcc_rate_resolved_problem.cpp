#include "multi_purpose_mpc_ros/mpcc_rate_resolved_problem.hpp"

#include <cmath>
#include <limits>
#include <utility>

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

bool valid_progress_aligned_wall_constraints(
  const ProgressAlignedWallConstraints & wall, const int horizon) noexcept
{
  const auto expected = static_cast<std::size_t>(horizon);
  if (
    wall.lower_slope.size() != expected ||
    wall.lower_intercept.size() != expected ||
    wall.upper_slope.size() != expected ||
    wall.upper_intercept.size() != expected)
  {
    return false;
  }
  for (std::size_t stage = 0U; stage < expected; ++stage) {
    if (
      !std::isfinite(wall.lower_slope[stage]) ||
      !std::isfinite(wall.upper_slope[stage]) ||
      std::isnan(wall.lower_intercept[stage]) ||
      std::isnan(wall.upper_intercept[stage]))
    {
      return false;
    }
  }
  return true;
}

bool valid_swept_lateral_wall_constraints(
  const std::vector<SweptLateralWallConstraint> & constraints,
  const int horizon) noexcept
{
  for (const auto & constraint : constraints) {
    if (
      constraint.transition_stage < 0 ||
      constraint.transition_stage >= horizon ||
      !std::isfinite(constraint.destination_ratio) ||
      constraint.destination_ratio <= 0.0 ||
      constraint.destination_ratio >= 1.0 ||
      std::isnan(constraint.lower_m) ||
      std::isnan(constraint.upper_m) ||
      constraint.lower_m > constraint.upper_m)
    {
      return false;
    }
  }
  return true;
}

bool valid_dynamic_obstacle_constraints(
  const std::vector<DynamicObstacleConstraint> & constraints,
  const int horizon) noexcept
{
  for (const auto & constraint : constraints) {
    const bool supported_axis =
      constraint.axis == DynamicObstacleConstraintAxis::Lateral ||
      constraint.axis == DynamicObstacleConstraintAxis::EffectiveProgress;
    if (
      constraint.state_stage <= 0 || constraint.state_stage > horizon ||
      !supported_axis ||
      std::isnan(constraint.lower) || std::isnan(constraint.upper) ||
      constraint.lower > constraint.upper)
    {
      return false;
    }
  }
  return true;
}

}  // namespace

std::optional<Eigen::Matrix<double, model::kInputDimension, 1>>
resolve_serialized_previous_input(
  const SerializedPreviousInputRequest & request) noexcept
{
  if (
    !std::isfinite(request.acceleration_mps2) ||
    !std::isfinite(request.virtual_progress_speed_mps) ||
    request.virtual_progress_speed_mps < 0.0 ||
    !std::isfinite(request.previous_steering_rad) ||
    !std::isfinite(request.published_steering_rad) ||
    !std::isfinite(request.publication_interval_sec) ||
    request.publication_interval_sec <= 0.0)
  {
    return std::nullopt;
  }
  Eigen::Matrix<double, model::kInputDimension, 1> previous_input;
  previous_input[model::kAccelerationIndex] = request.acceleration_mps2;
  previous_input[model::kSteeringRateIndex] =
    (request.published_steering_rad - request.previous_steering_rad) /
    request.publication_interval_sec;
  previous_input[model::kVirtualProgressSpeedIndex] =
    request.virtual_progress_speed_mps;
  if (!previous_input.allFinite()) {
    return std::nullopt;
  }
  return previous_input;
}

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
  const int steering_prefix_rows =
    request.steering_rate_prefix_bounds.has_value() ? horizon : 0;
  const int progress_wall_rows =
    request.progress_aligned_wall_constraints.has_value() ? 2 * horizon : 0;
  const int swept_wall_rows = static_cast<int>(
    request.swept_lateral_wall_constraints.size());
  const int dynamic_obstacle_rows = static_cast<int>(
    request.dynamic_obstacle_constraints.size());
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
    (request.additional_linear_cost.size() != 0 &&
    request.additional_linear_cost.size() != variable_count) ||
    !request.state_reference.allFinite() ||
    !request.input_reference.allFinite() ||
    !finite_non_negative(request.state_weight) ||
    !finite_non_negative(request.input_weight) ||
    (request.additional_linear_cost.size() != 0 &&
    !request.additional_linear_cost.allFinite()) ||
    !request.previous_input.allFinite() ||
    !request.input_delta_weight.allFinite() ||
    (request.input_delta_weight.array() < 0.0).any() ||
    !valid_bounds(request.state_lower, request.state_upper) ||
    !valid_bounds(request.input_lower, request.input_upper) ||
    (request.steering_rate_prefix_bounds.has_value() &&
    (!std::isfinite(
      request.steering_rate_prefix_bounds->minimum_cumulative_delta_rad) ||
    !std::isfinite(
      request.steering_rate_prefix_bounds->maximum_cumulative_delta_rad) ||
    request.steering_rate_prefix_bounds->minimum_cumulative_delta_rad >
    request.steering_rate_prefix_bounds->maximum_cumulative_delta_rad)) ||
    (request.progress_aligned_wall_constraints.has_value() &&
    !valid_progress_aligned_wall_constraints(
      request.progress_aligned_wall_constraints.value(), horizon)) ||
    !valid_swept_lateral_wall_constraints(
      request.swept_lateral_wall_constraints, horizon) ||
    !valid_dynamic_obstacle_constraints(
      request.dynamic_obstacle_constraints, horizon))
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
  const int steering_prefix_offset = state_values + variable_count;
  if (request.steering_rate_prefix_bounds.has_value()) {
    for (int stage = 0; stage < horizon; ++stage) {
      for (int prefix_stage = 0; prefix_stage <= stage; ++prefix_stage) {
        constraint_triplets.emplace_back(
          steering_prefix_offset + stage,
          state_values + prefix_stage * nu + model::kSteeringRateIndex,
          request.linearizations[static_cast<std::size_t>(prefix_stage)].
          stage_dt_sec);
      }
    }
  }
  const int progress_wall_offset = steering_prefix_offset + steering_prefix_rows;
  if (request.progress_aligned_wall_constraints.has_value()) {
    const auto & wall = request.progress_aligned_wall_constraints.value();
    for (int stage = 0; stage < horizon; ++stage) {
      const int state = (stage + 1) * nx;
      const int lower_row = progress_wall_offset + 2 * stage;
      const int upper_row = lower_row + 1;
      const auto index = static_cast<std::size_t>(stage);
      constraint_triplets.emplace_back(
        lower_row, state + model::kLateralIndex, 1.0);
      constraint_triplets.emplace_back(
        lower_row, state + model::kProgressIndex,
        -wall.lower_slope[index]);
      constraint_triplets.emplace_back(
        upper_row, state + model::kLateralIndex, 1.0);
      constraint_triplets.emplace_back(
        upper_row, state + model::kProgressIndex,
        -wall.upper_slope[index]);
    }
  }
  const int swept_wall_offset = progress_wall_offset + progress_wall_rows;
  for (std::size_t index = 0U;
    index < request.swept_lateral_wall_constraints.size(); ++index)
  {
    const auto & wall = request.swept_lateral_wall_constraints[index];
    const int row = swept_wall_offset + static_cast<int>(index);
    const int source_state = wall.transition_stage * nx;
    const int destination_state = (wall.transition_stage + 1) * nx;
    constraint_triplets.emplace_back(
      row, source_state + model::kLateralIndex,
      1.0 - wall.destination_ratio);
    constraint_triplets.emplace_back(
      row, destination_state + model::kLateralIndex,
      wall.destination_ratio);
  }
  const int dynamic_obstacle_offset = swept_wall_offset + swept_wall_rows;
  for (std::size_t index = 0U;
    index < request.dynamic_obstacle_constraints.size(); ++index)
  {
    const auto & obstacle = request.dynamic_obstacle_constraints[index];
    const int row = dynamic_obstacle_offset + static_cast<int>(index);
    const int state = obstacle.state_stage * nx;
    if (obstacle.axis == DynamicObstacleConstraintAxis::Lateral) {
      constraint_triplets.emplace_back(
        row, state + model::kLateralIndex, 1.0);
    } else {
      // Physical along-track position is theta + e_lag.  A theta-only row
      // disagrees with the canonical Follow gap certificate and can turn a
      // safe negative-lag state into a false longitudinal collision.
      constraint_triplets.emplace_back(
        row, state + model::kProgressIndex, 1.0);
      constraint_triplets.emplace_back(
        row, state + model::kLagIndex, 1.0);
    }
  }
  Eigen::SparseMatrix<double> constraints(
    state_values + variable_count + steering_prefix_rows + progress_wall_rows +
    swept_wall_rows + dynamic_obstacle_rows,
    variable_count);
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
  Eigen::VectorXd lower_bound(
    state_values + variable_count + steering_prefix_rows + progress_wall_rows +
    swept_wall_rows + dynamic_obstacle_rows);
  Eigen::VectorXd upper_bound(
    state_values + variable_count + steering_prefix_rows + progress_wall_rows +
    swept_wall_rows + dynamic_obstacle_rows);
  lower_bound.head(state_values + variable_count) << equality, box_lower;
  upper_bound.head(state_values + variable_count) << equality, box_upper;
  if (request.steering_rate_prefix_bounds.has_value()) {
    lower_bound.segment(steering_prefix_offset, steering_prefix_rows).setConstant(
      request.steering_rate_prefix_bounds->minimum_cumulative_delta_rad);
    upper_bound.segment(steering_prefix_offset, steering_prefix_rows).setConstant(
      request.steering_rate_prefix_bounds->maximum_cumulative_delta_rad);
  }
  if (request.progress_aligned_wall_constraints.has_value()) {
    const auto & wall = request.progress_aligned_wall_constraints.value();
    for (int stage = 0; stage < horizon; ++stage) {
      const auto index = static_cast<std::size_t>(stage);
      const int lower_row = progress_wall_offset + 2 * stage;
      const int upper_row = lower_row + 1;
      lower_bound[lower_row] = wall.lower_intercept[index];
      upper_bound[lower_row] = std::numeric_limits<double>::infinity();
      lower_bound[upper_row] = -std::numeric_limits<double>::infinity();
      upper_bound[upper_row] = wall.upper_intercept[index];
    }
  }
  for (std::size_t index = 0U;
    index < request.swept_lateral_wall_constraints.size(); ++index)
  {
    const int row = swept_wall_offset + static_cast<int>(index);
    lower_bound[row] = request.swept_lateral_wall_constraints[index].lower_m;
    upper_bound[row] = request.swept_lateral_wall_constraints[index].upper_m;
  }
  for (std::size_t index = 0U;
    index < request.dynamic_obstacle_constraints.size(); ++index)
  {
    const int row = dynamic_obstacle_offset + static_cast<int>(index);
    lower_bound[row] = request.dynamic_obstacle_constraints[index].lower;
    upper_bound[row] = request.dynamic_obstacle_constraints[index].upper;
  }

  Eigen::VectorXd linear_cost = Eigen::VectorXd::Zero(variable_count);
  if (request.additional_linear_cost.size() != 0) {
    linear_cost = request.additional_linear_cost;
  }
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

RowSemantic decode_row(
  const int row, const int horizon_steps,
  const bool steering_rate_prefix_active,
  const bool progress_aligned_wall_active,
  const int swept_lateral_wall_count,
  const std::vector<DynamicObstacleConstraint> * dynamic_obstacle_constraints) noexcept
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
  int trailing_row = input_row - input_values;
  if (steering_rate_prefix_active && trailing_row < horizon_steps) {
    return RowSemantic{
      true, RowKind::SteeringRatePrefix, trailing_row,
      model::kSteeringRateIndex};
  }
  if (steering_rate_prefix_active) {
    trailing_row -= horizon_steps;
  }
  if (progress_aligned_wall_active && trailing_row < 2 * horizon_steps) {
    return RowSemantic{
      true,
      trailing_row % 2 == 0 ? RowKind::ProgressAlignedWallLower :
      RowKind::ProgressAlignedWallUpper,
      trailing_row / 2, model::kLateralIndex};
  }
  if (progress_aligned_wall_active) {
    trailing_row -= 2 * horizon_steps;
  }
  if (
    swept_lateral_wall_count > 0 && trailing_row >= 0 &&
    trailing_row < swept_lateral_wall_count)
  {
    return RowSemantic{
      true, RowKind::SweptLateralWall, trailing_row,
      model::kLateralIndex};
  }
  if (swept_lateral_wall_count > 0) {
    trailing_row -= swept_lateral_wall_count;
  }
  if (
    dynamic_obstacle_constraints != nullptr && trailing_row >= 0 &&
    trailing_row < static_cast<int>(dynamic_obstacle_constraints->size()))
  {
    const auto & obstacle = (*dynamic_obstacle_constraints)[
      static_cast<std::size_t>(trailing_row)];
    return RowSemantic{
      true,
      obstacle.axis == DynamicObstacleConstraintAxis::Lateral ?
      RowKind::DynamicObstacleLateral :
      RowKind::DynamicObstacleEffectiveProgress,
      obstacle.state_stage,
      obstacle.axis == DynamicObstacleConstraintAxis::Lateral ?
      model::kLateralIndex : model::kProgressIndex};
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
    case RowKind::SteeringRatePrefix: return "steering-rate-prefix";
    case RowKind::ProgressAlignedWallLower:
      return "progress-aligned-wall-lower";
    case RowKind::ProgressAlignedWallUpper:
      return "progress-aligned-wall-upper";
    case RowKind::SweptLateralWall: return "swept-lateral-wall";
    case RowKind::DynamicObstacleLateral:
      return "dynamic-obstacle-lateral";
    case RowKind::DynamicObstacleEffectiveProgress:
      return "dynamic-obstacle-effective-progress";
  }
  return "unknown";
}

FirstStageInputFeasibility analyze_first_stage_input_feasibility(
  const AssemblyRequest & request, const int input_element) noexcept
{
  constexpr int nx = model::kStateDimension;
  constexpr int nu = model::kInputDimension;
  FirstStageInputFeasibility result;
  result.input_element = input_element;
  if (
    request.horizon_steps <= 0 || input_element < 0 || input_element >= nu ||
    !request.initial_state.allFinite() || request.linearizations.empty() ||
    request.state_lower.size() < 2 * nx || request.state_upper.size() < 2 * nx ||
    request.input_lower.size() < nu || request.input_upper.size() < nu)
  {
    return result;
  }
  const auto & linearization = request.linearizations.front();
  if (
    !linearization.state_matrix.allFinite() ||
    !linearization.input_matrix.allFinite() ||
    !linearization.equality_offset.allFinite())
  {
    return result;
  }
  result.evaluated = true;
  result.separable = true;
  result.declared_lower = request.input_lower[input_element];
  result.declared_upper = request.input_upper[input_element];
  result.implied_lower = result.declared_lower;
  result.implied_upper = result.declared_upper;
  constexpr double coefficient_tolerance = 1e-12;
  for (int state_element = 0; state_element < nx; ++state_element) {
    const double coefficient =
      linearization.input_matrix(state_element, input_element);
    if (std::abs(coefficient) <= coefficient_tolerance) {
      continue;
    }
    bool coupled = false;
    for (int other_input = 0; other_input < nu; ++other_input) {
      if (
        other_input != input_element &&
        std::abs(linearization.input_matrix(state_element, other_input)) >
        coefficient_tolerance)
      {
        result.separable = false;
        coupled = true;
        break;
      }
    }
    if (coupled) {
      continue;
    }
    const double base =
      linearization.state_matrix.row(state_element).dot(request.initial_state) -
      linearization.equality_offset[state_element];
    const double state_lower = request.state_lower[nx + state_element];
    const double state_upper = request.state_upper[nx + state_element];
    double input_lower = (state_lower - base) / coefficient;
    double input_upper = (state_upper - base) / coefficient;
    if (coefficient < 0.0) {
      std::swap(input_lower, input_upper);
    }
    if (input_lower > result.implied_lower) {
      result.implied_lower = input_lower;
      result.limiting_lower_state_element = state_element;
    }
    if (input_upper < result.implied_upper) {
      result.implied_upper = input_upper;
      result.limiting_upper_state_element = state_element;
    }
  }
  const bool separable_interval_nonempty =
    !std::isnan(result.implied_lower) && !std::isnan(result.implied_upper) &&
    result.implied_lower <= result.implied_upper;
  result.conclusive = result.separable || !separable_interval_nonempty;
  result.feasible = result.conclusive && separable_interval_nonempty;
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_problem
