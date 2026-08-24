#include "multi_purpose_mpc_ros/mpcc_rate_resolved_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_adapter
{
namespace
{

template<typename Vector>
bool valid_bounds(const Vector & lower, const Vector & upper) noexcept
{
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

bool valid_state_stage(const StateStage & stage) noexcept
{
  return stage.reference.allFinite() && stage.weight.allFinite() &&
    stage.linear_cost.allFinite() &&
    (stage.weight.array() >= 0.0).all() &&
    valid_bounds(stage.lower, stage.upper);
}

bool valid_input_stage(const InputStage & stage) noexcept
{
  return stage.reference.allFinite() && stage.weight.allFinite() &&
    stage.linear_cost.allFinite() &&
    (stage.weight.array() >= 0.0).all() &&
    valid_bounds(stage.lower, stage.upper) &&
    stage.linear_cost[kLegacyCurvatureIndex] == 0.0 &&
    std::isfinite(stage.lower[kLegacyCurvatureIndex]) &&
    std::isfinite(stage.upper[kLegacyCurvatureIndex]) &&
    std::isfinite(stage.path_curvature_radpm) &&
    std::isfinite(stage.stage_dt_sec);
}

double steering_from_curvature(
  const double wheelbase_m, const double curvature_radpm) noexcept
{
  return std::atan(wheelbase_m * curvature_radpm);
}

double curvature_jacobian(
  const double wheelbase_m, const double steering_rad) noexcept
{
  const double cosine = std::cos(steering_rad);
  return 1.0 / (wheelbase_m * cosine * cosine);
}

}  // namespace

std::optional<Result> build(const Request & request) noexcept
{
  namespace model = mpcc_rate_resolved;
  constexpr double half_pi = 1.57079632679489661923;
  const int horizon = request.horizon_steps;
  if (
    horizon <= 0 || !request.initial_state.allFinite() ||
    !std::isfinite(request.current_steering_rad) ||
    !std::isfinite(request.wheelbase_m) || request.wheelbase_m <= 0.0 ||
    !std::isfinite(request.maximum_abs_steering_rad) ||
    request.maximum_abs_steering_rad <= 0.0 ||
    request.maximum_abs_steering_rad >= half_pi ||
    std::abs(request.current_steering_rad) >
    request.maximum_abs_steering_rad ||
    !std::isfinite(request.maximum_abs_steering_rate_radps) ||
    request.maximum_abs_steering_rate_radps < 0.0 ||
    !std::isfinite(request.minimum_frenet_denominator) ||
    request.minimum_frenet_denominator <= 0.0 ||
    !std::isfinite(request.minimum_stage_dt_sec) ||
    request.minimum_stage_dt_sec <= 0.0 ||
    !std::isfinite(request.maximum_stage_dt_sec) ||
    request.maximum_stage_dt_sec < request.minimum_stage_dt_sec ||
    request.states.size() != static_cast<std::size_t>(horizon + 1) ||
    request.inputs.size() != static_cast<std::size_t>(horizon) ||
    !request.previous_input.allFinite() ||
    !request.input_delta_weight.allFinite() ||
    (request.input_delta_weight.array() < 0.0).any())
  {
    return std::nullopt;
  }
  for (const auto & stage : request.states) {
    if (!valid_state_stage(stage)) {
      return std::nullopt;
    }
  }
  for (const auto & stage : request.inputs) {
    if (
      !valid_input_stage(stage) ||
      stage.stage_dt_sec < request.minimum_stage_dt_sec ||
      stage.stage_dt_sec > request.maximum_stage_dt_sec)
    {
      return std::nullopt;
    }
  }
  for (int element = 0; element < kLegacyStateDimension; ++element) {
    if (
      request.initial_state[element] < request.states.front().lower[element] ||
      request.initial_state[element] > request.states.front().upper[element])
    {
      return std::nullopt;
    }
  }

  const int state_values = model::kStateDimension * (horizon + 1);
  const int input_values = model::kInputDimension * horizon;
  Result result;
  auto & problem = result.problem;
  problem.horizon_steps = horizon;
  problem.initial_state.head<kLegacyStateDimension>() = request.initial_state;
  problem.initial_state[model::kSteeringIndex] = request.current_steering_rad;
  problem.state_reference = Eigen::VectorXd::Zero(state_values);
  problem.state_lower = Eigen::VectorXd::Zero(state_values);
  problem.state_upper = Eigen::VectorXd::Zero(state_values);
  problem.state_weight = Eigen::VectorXd::Zero(state_values);
  problem.input_reference = Eigen::VectorXd::Zero(input_values);
  problem.input_lower = Eigen::VectorXd::Zero(input_values);
  problem.input_upper = Eigen::VectorXd::Zero(input_values);
  problem.input_weight = Eigen::VectorXd::Zero(input_values);
  problem.additional_linear_cost = Eigen::VectorXd::Zero(
    state_values + input_values);
  problem.previous_input <<
    request.previous_input[0], 0.0, request.previous_input[2];
  problem.input_delta_weight <<
    request.input_delta_weight[0], 0.0, request.input_delta_weight[2];

  result.steering_reference_rad.resize(static_cast<std::size_t>(horizon + 1));
  result.steering_lower_rad.resize(static_cast<std::size_t>(horizon + 1));
  result.steering_upper_rad.resize(static_cast<std::size_t>(horizon + 1));
  result.curvature_to_steering_jacobian_radpm_per_rad.resize(
    static_cast<std::size_t>(horizon));

  for (int stage = 0; stage <= horizon; ++stage) {
    const int state_offset = model::kStateDimension * stage;
    problem.state_reference.segment<kLegacyStateDimension>(state_offset) =
      stage == 0 ? request.initial_state :
      request.states[static_cast<std::size_t>(stage)].reference;
    problem.state_lower.segment<kLegacyStateDimension>(state_offset) =
      request.states[static_cast<std::size_t>(stage)].lower;
    problem.state_upper.segment<kLegacyStateDimension>(state_offset) =
      request.states[static_cast<std::size_t>(stage)].upper;
    problem.state_weight.segment<kLegacyStateDimension>(state_offset) =
      request.states[static_cast<std::size_t>(stage)].weight;
    problem.additional_linear_cost.segment<kLegacyStateDimension>(state_offset) =
      request.states[static_cast<std::size_t>(stage)].linear_cost;

    const int source_input = std::max(0, stage - 1);
    const double steering_reference = stage == 0 ?
      request.current_steering_rad :
      steering_from_curvature(
      request.wheelbase_m,
      request.inputs[static_cast<std::size_t>(source_input)].
      reference[kLegacyCurvatureIndex]);
    const double curvature_lower = stage == 0 ?
      -std::numeric_limits<double>::infinity() :
      request.inputs[static_cast<std::size_t>(source_input)].
      lower[kLegacyCurvatureIndex];
    const double curvature_upper = stage == 0 ?
      std::numeric_limits<double>::infinity() :
      request.inputs[static_cast<std::size_t>(source_input)].
      upper[kLegacyCurvatureIndex];
    const double steering_lower = std::max(
      -request.maximum_abs_steering_rad,
      steering_from_curvature(request.wheelbase_m, curvature_lower));
    const double steering_upper = std::min(
      request.maximum_abs_steering_rad,
      steering_from_curvature(request.wheelbase_m, curvature_upper));
    if (
      !std::isfinite(steering_reference) ||
      !std::isfinite(steering_lower) || !std::isfinite(steering_upper) ||
      steering_lower > steering_upper)
    {
      return std::nullopt;
    }
    problem.state_reference[state_offset + model::kSteeringIndex] =
      steering_reference;
    problem.state_lower[state_offset + model::kSteeringIndex] = steering_lower;
    problem.state_upper[state_offset + model::kSteeringIndex] = steering_upper;
    result.steering_reference_rad[static_cast<std::size_t>(stage)] =
      steering_reference;
    result.steering_lower_rad[static_cast<std::size_t>(stage)] = steering_lower;
    result.steering_upper_rad[static_cast<std::size_t>(stage)] = steering_upper;
    if (stage > 0) {
      const auto & input = request.inputs[static_cast<std::size_t>(source_input)];
      const double jacobian = curvature_jacobian(
        request.wheelbase_m, steering_reference);
      if (!std::isfinite(jacobian) || jacobian <= 0.0) {
        return std::nullopt;
      }
      problem.state_weight[state_offset + model::kSteeringIndex] =
        input.weight[kLegacyCurvatureIndex] * jacobian * jacobian;
    }
  }

  problem.linearizations.reserve(static_cast<std::size_t>(horizon));
  for (int stage = 0; stage < horizon; ++stage) {
    const auto index = static_cast<std::size_t>(stage);
    const auto & legacy_state = request.states[index];
    const auto & legacy_input = request.inputs[index];
    const auto state_reference = stage == 0 ?
      request.initial_state : legacy_state.reference;
    const int input_offset = model::kInputDimension * stage;
    const double steering_reference =
      result.steering_reference_rad[index];
    const auto linearization = model::linearize_temporal_frenet(
      model::LinearizationRequest{
        state_reference[0], state_reference[1], state_reference[2],
        state_reference[3], state_reference[4], steering_reference,
        legacy_input.reference[0], 0.0, legacy_input.reference[2],
        legacy_input.path_curvature_radpm, request.wheelbase_m,
        legacy_input.stage_dt_sec, request.minimum_frenet_denominator,
        request.minimum_stage_dt_sec, request.maximum_stage_dt_sec});
    if (!linearization.has_value()) {
      return std::nullopt;
    }
    problem.linearizations.push_back(linearization.value());

    problem.input_reference[input_offset + model::kAccelerationIndex] =
      legacy_input.reference[0];
    problem.input_reference[input_offset + model::kSteeringRateIndex] = 0.0;
    problem.input_reference[
      input_offset + model::kVirtualProgressSpeedIndex] =
      legacy_input.reference[2];
    problem.input_lower[input_offset + model::kAccelerationIndex] =
      legacy_input.lower[0];
    problem.input_upper[input_offset + model::kAccelerationIndex] =
      legacy_input.upper[0];
    problem.input_lower[input_offset + model::kSteeringRateIndex] =
      -request.maximum_abs_steering_rate_radps;
    problem.input_upper[input_offset + model::kSteeringRateIndex] =
      request.maximum_abs_steering_rate_radps;
    problem.input_lower[
      input_offset + model::kVirtualProgressSpeedIndex] =
      legacy_input.lower[2];
    problem.input_upper[
      input_offset + model::kVirtualProgressSpeedIndex] =
      legacy_input.upper[2];
    problem.input_weight[input_offset + model::kAccelerationIndex] =
      legacy_input.weight[0];
    problem.input_weight[
      input_offset + model::kVirtualProgressSpeedIndex] =
      legacy_input.weight[2];
    const int linear_input_offset = state_values + input_offset;
    problem.additional_linear_cost[
      linear_input_offset + model::kAccelerationIndex] =
      legacy_input.linear_cost[0];
    problem.additional_linear_cost[
      linear_input_offset + model::kVirtualProgressSpeedIndex] =
      legacy_input.linear_cost[2];

    const double jacobian = curvature_jacobian(
      request.wheelbase_m, steering_reference);
    result.curvature_to_steering_jacobian_radpm_per_rad[index] = jacobian;
    const double curvature_change_weight =
      request.input_delta_weight[kLegacyCurvatureIndex];
    problem.input_weight[input_offset + model::kSteeringRateIndex] =
      curvature_change_weight * jacobian * jacobian *
      legacy_input.stage_dt_sec * legacy_input.stage_dt_sec;
  }
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_adapter
