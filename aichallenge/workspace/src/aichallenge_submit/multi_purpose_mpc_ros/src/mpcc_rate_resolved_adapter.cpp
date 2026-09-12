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


// A tangent owns a transition, so its state and virtual speed must jointly
// lie in the immutable course domain. Soft references and QP residuals can
// both violate this domain even when each independent box is respected.
// Select only the tangent input; never rewrite costs, bounds or a solution.
std::optional<double> select_virtual_speed_tangent(
  const mpcc_rate_resolved::CourseFrame & frame,
  const double progress, const double dt,
  const double reference, const double lower, const double upper) noexcept
{
  if (!frame.knots) {
    return reference;
  }
  if (frame.knots->size() < 2U || !std::isfinite(dt) || dt <= 0.0 ||
    !std::isfinite(progress) || !std::isfinite(reference))
  {
    return std::nullopt;
  }
  const double minimum = std::max(lower,
    (frame.knots->front().progress_m - frame.progress_origin_m - progress) / dt);
  const double maximum = std::min(upper,
    (frame.knots->back().progress_m - frame.progress_origin_m - progress) / dt);
  if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum > maximum) {
    return std::nullopt;
  }
  return std::clamp(reference, minimum, maximum);
}

double steering_from_curvature(
  const double wheelbase_m, const double curvature_reference_gain,
  const double curvature_radpm) noexcept
{
  // Preserve the existing curvature-reference and command-bound conversion.
  // This soft-reference calibration is independent of the body/tire dynamics.
  return std::atan(wheelbase_m * curvature_radpm / curvature_reference_gain);
}

double curvature_jacobian(
  const double wheelbase_m, const double curvature_reference_gain,
  const double steering_rad) noexcept
{
  const double cosine = std::cos(steering_rad);
  return curvature_reference_gain / (wheelbase_m * cosine * cosine);
}

}  // namespace

std::optional<ExactPhysicalBoundaryBounds>
resolve_exact_physical_boundary_bounds(
  const double physical_lower, const double physical_upper,
  const persistent_osqp::PhysicalConstraintTolerance & tolerance) noexcept
{
  const double accepted_absolute =
    persistent_osqp::kSolvedInaccurateToleranceMultiplier *
    tolerance.absolute;
  const double accepted_relative =
    persistent_osqp::kSolvedInaccurateToleranceMultiplier *
    tolerance.relative;
  if (
    std::isnan(physical_lower) || std::isnan(physical_upper) ||
    physical_lower > physical_upper || !std::isfinite(accepted_absolute) ||
    accepted_absolute < 0.0 || !std::isfinite(accepted_relative) ||
    accepted_relative < 0.0 || accepted_relative >= 1.0)
  {
    return std::nullopt;
  }
  double characteristic = 0.0;
  if (std::isfinite(physical_lower)) {
    characteristic = std::max(characteristic, std::abs(physical_lower));
  }
  if (std::isfinite(physical_upper)) {
    characteristic = std::max(characteristic, std::abs(physical_upper));
  }
  const double margin =
    (accepted_absolute + accepted_relative * characteristic) /
    (1.0 - accepted_relative);
  const double solver_lower = std::isfinite(physical_lower) ?
    physical_lower + margin : physical_lower;
  const double solver_upper = std::isfinite(physical_upper) ?
    physical_upper - margin : physical_upper;
  if (
    !std::isfinite(margin) || margin < 0.0 ||
    std::isnan(solver_lower) || std::isnan(solver_upper) ||
    solver_lower > solver_upper)
  {
    return std::nullopt;
  }
  return ExactPhysicalBoundaryBounds{solver_lower, solver_upper, margin};
}

std::optional<ExactPhysicalBoundaryBounds> resolve_steering_prefix_bounds(
  const double initial_steering_rad, const double maximum_abs_steering_rad,
  const persistent_osqp::PhysicalConstraintTolerance & tolerance) noexcept
{
  if (!std::isfinite(initial_steering_rad) ||
    !std::isfinite(maximum_abs_steering_rad) || maximum_abs_steering_rad <= 0.0)
  {
    return std::nullopt;
  }
  return resolve_exact_physical_boundary_bounds(
    -maximum_abs_steering_rad - initial_steering_rad,
    maximum_abs_steering_rad - initial_steering_rad, tolerance);
}

const char * to_string(const RejectReason reason) noexcept
{
  switch (reason) {
    case RejectReason::None: return "none";
    case RejectReason::InvalidRequest: return "invalid-request";
    case RejectReason::InvalidStateStage: return "invalid-state-stage";
    case RejectReason::InvalidInputStage: return "invalid-input-stage";
    case RejectReason::InitialStateOutsideBounds:
      return "initial-state-outside-bounds";
    case RejectReason::SteeringBoundsUnavailable:
      return "steering-bounds-unavailable";
    case RejectReason::SteeringJacobianUnavailable:
      return "steering-jacobian-unavailable";
    case RejectReason::LinearizationUnavailable:
      return "linearization-unavailable";
    case RejectReason::AccelerationInsetUnavailable:
      return "acceleration-inset-unavailable";
    case RejectReason::SteeringRateInsetUnavailable:
      return "steering-rate-inset-unavailable";
    case RejectReason::SteeringPrefixInsetUnavailable:
      return "steering-prefix-inset-unavailable";
  }
  return "unknown";
}

const char * to_string(const RelinearizationReason reason) noexcept
{
  switch (reason) {
    case RelinearizationReason::Accepted: return "accepted";
    case RelinearizationReason::InvalidRequest: return "invalid-request";
    case RelinearizationReason::InvalidPrimal: return "invalid-primal";
    case RelinearizationReason::LinearizationUnavailable:
      return "linearization-unavailable";
  }
  return "unknown";
}

bool is_braking_feasibility_request(const Request & request) noexcept
{
  namespace model = mpcc_rate_resolved;
  if (request.horizon_steps <= 0 ||
    request.inputs.size() != static_cast<std::size_t>(request.horizon_steps) ||
    request.states.size() != request.inputs.size() + 1U ||
    !request.input_delta_weight.isZero(0.0))
  {
    return false;
  }
  for (const auto & state : request.states) {
    if (!state.weight.isZero(0.0) || !state.linear_cost.isZero(0.0)) {
      return false;
    }
  }
  if (!request.maximum_braking_feasibility ||
    !mpcc_vehicle_model::valid(request.vehicle_model)) return false;
  mpcc_vehicle_model::State body{0, 0, 0, request.initial_state[model::kVelocityIndex],
    request.current_lateral_velocity_mps, request.current_yaw_rate_radps,
    request.current_steering_rad, request.current_response_steering_rad};
  std::size_t stage = 0U;
  for (const auto & input : request.inputs) {
    if (!input.weight.isZero(0.0) || !input.linear_cost.isZero(0.0) ||
      !std::isfinite(input.stage_dt_sec) || input.stage_dt_sec <= 0.0 ||
      !std::isfinite(input.reference[model::kAccelerationIndex]) ||
      input.reference[model::kAccelerationIndex] >= 0.0 ||
      input.reference[model::kAccelerationIndex] < input.lower[model::kAccelerationIndex] ||
      input.reference[model::kAccelerationIndex] > input.upper[model::kAccelerationIndex])
    {
      return false;
    }
    const auto next = mpcc_vehicle_model::advance(
      body, {input.reference[model::kAccelerationIndex], 0.0},
      request.vehicle_model, input.stage_dt_sec);
    if (!next || !std::isfinite(request.states[stage + 1U].reference[model::kVelocityIndex]) ||
      std::abs(next->state.forward_velocity_mps -
      request.states[stage + 1U].reference[model::kVelocityIndex]) > 1e-12) return false;
    body = next->state;
    ++stage;
  }
  const auto & terminal = request.states.back();
  return terminal.lower[model::kVelocityIndex] == 0.0 &&
         terminal.upper[model::kVelocityIndex] == 0.0;

}

std::optional<Result> build(
  const Request & request,
  const persistent_osqp::PhysicalConstraintTolerance & solver_tolerance,
  BuildDiagnostic * diagnostic) noexcept
{
  namespace model = mpcc_rate_resolved;
  constexpr double half_pi = 1.57079632679489661923;
  if (diagnostic != nullptr) {
    *diagnostic = BuildDiagnostic{};
  }
  const auto reject = [diagnostic](
    const RejectReason reason, const int stage = -1, const int element = -1,
    const double value = std::numeric_limits<double>::quiet_NaN(),
    const double lower = std::numeric_limits<double>::quiet_NaN(),
    const double upper = std::numeric_limits<double>::quiet_NaN())
    -> std::optional<Result>
    {
      if (diagnostic != nullptr) {
        *diagnostic = BuildDiagnostic{
          reason, stage, element, value, lower, upper};
      }
      return std::nullopt;
    };
  const int horizon = request.horizon_steps;
  if (
    horizon <= 0 || !request.initial_state.allFinite() ||
    !std::isfinite(request.current_steering_rad) ||
    !std::isfinite(request.current_response_steering_rad) ||
    !std::isfinite(request.current_lateral_velocity_mps) ||
    !std::isfinite(request.current_yaw_rate_radps) ||
    !mpcc_vehicle_model::valid(request.vehicle_model) ||
    !std::isfinite(request.wheelbase_m) || request.wheelbase_m <= 0.0 ||
    !std::isfinite(request.curvature_reference_gain) ||
    request.curvature_reference_gain <= 0.0 ||
    !std::isfinite(request.maximum_abs_steering_rad) ||
    request.maximum_abs_steering_rad <= 0.0 ||
    request.maximum_abs_steering_rad >= half_pi ||
    std::abs(request.current_steering_rad) >
    request.maximum_abs_steering_rad ||
    std::abs(request.current_response_steering_rad) >
    request.maximum_abs_steering_rad ||
    !std::isfinite(request.maximum_abs_steering_rate_radps) ||
    request.maximum_abs_steering_rate_radps < 0.0 ||
    !std::isfinite(solver_tolerance.absolute) ||
    solver_tolerance.absolute < 0.0 ||
    !std::isfinite(solver_tolerance.relative) ||
    solver_tolerance.relative < 0.0 || solver_tolerance.relative >= 1.0 ||
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
    return reject(RejectReason::InvalidRequest);
  }
  for (std::size_t stage = 0U; stage < request.states.size(); ++stage) {
    if (!valid_state_stage(request.states[stage])) {
      return reject(
        RejectReason::InvalidStateStage, static_cast<int>(stage));
    }
  }
  for (std::size_t stage = 0U; stage < request.inputs.size(); ++stage) {
    const auto & input = request.inputs[stage];
    if (
      !valid_input_stage(input) ||
      input.stage_dt_sec < request.minimum_stage_dt_sec ||
      input.stage_dt_sec > request.maximum_stage_dt_sec)
    {
      return reject(
        RejectReason::InvalidInputStage, static_cast<int>(stage));
    }
  }
  const int state_values = model::kStateDimension * (horizon + 1);
  const int input_values = model::kInputDimension * horizon;
  Result result;
  auto & problem = result.problem;
  problem.horizon_steps = horizon;
  problem.initial_state.head<kLegacyStateDimension>() = request.initial_state;
  problem.initial_state[model::kSteeringIndex] = request.current_steering_rad;
  problem.initial_state[model::kResponseSteeringIndex] =
    request.current_response_steering_rad;
  problem.initial_state[model::kLateralVelocityIndex] = request.current_lateral_velocity_mps;
  problem.initial_state[model::kYawRateIndex] = request.current_yaw_rate_radps;
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
  std::vector<double> response_steering_reference_rad(
    static_cast<std::size_t>(horizon + 1),
    request.current_response_steering_rad);
  std::vector<double> lateral_velocity_reference_mps(
    static_cast<std::size_t>(horizon + 1), request.current_lateral_velocity_mps);
  std::vector<double> yaw_rate_reference_radps(
    static_cast<std::size_t>(horizon + 1), request.current_yaw_rate_radps);
  result.curvature_to_steering_jacobian_radpm_per_rad.resize(
    static_cast<std::size_t>(horizon));
  // The physical steering state is the only origin of the optimized rate
  // prefix. Integrating that rate again from the previous desired command
  // creates a permanently offset trajectory outside the QP wall proof.
  const double physical_prefix_lower =
    -request.maximum_abs_steering_rad - request.current_steering_rad;
  const double physical_prefix_upper =
    request.maximum_abs_steering_rad - request.current_steering_rad;
  const auto solver_prefix_bounds = resolve_steering_prefix_bounds(
    request.current_steering_rad, request.maximum_abs_steering_rad, solver_tolerance);
  if (!solver_prefix_bounds.has_value()) {
    return reject(
      RejectReason::SteeringPrefixInsetUnavailable, -1,
      model::kSteeringRateIndex, 0.0,
      physical_prefix_lower, physical_prefix_upper);
  }
  problem.steering_rate_prefix_bounds =
    mpcc_rate_resolved_problem::SteeringRatePrefixBounds{
      solver_prefix_bounds->lower, solver_prefix_bounds->upper};

  for (int stage = 0; stage <= horizon; ++stage) {
    const int state_offset = model::kStateDimension * stage;
    problem.state_reference.segment<kLegacyStateDimension>(state_offset) =
      stage == 0 ? request.initial_state :
      request.states[static_cast<std::size_t>(stage)].reference;
    problem.state_lower.segment<kLegacyStateDimension>(state_offset) =
      request.states[static_cast<std::size_t>(stage)].lower;
    problem.state_upper.segment<kLegacyStateDimension>(state_offset) =
      request.states[static_cast<std::size_t>(stage)].upper;
    if (stage == 0) {
      // x(0) is the measured state and is already owned by the initial-state
      // equality.  Reapplying a conservative future wall/corridor box to the
      // same immutable value can only make the QP infeasible; it cannot move
      // the vehicle back into that box.  This previously rejected every
      // continuation, alternate branch and Stop candidate before any input
      // was optimized when tracking error placed the current pose just
      // outside the progress-aligned approximation.  Future boxes, swept
      // constraints and the downstream exact footprint certificate remain
      // unchanged and own physical acceptance.
      problem.state_lower.segment<kLegacyStateDimension>(state_offset) =
        request.initial_state;
      problem.state_upper.segment<kLegacyStateDimension>(state_offset) =
        request.initial_state;
    }
    // Future velocity is a predicted state, not a command crossing the
    // publisher boundary.  Its physical certificate already owns the solver
    // residual tolerance.  Applying the command inset here turns a legitimate
    // stop state [0, 0] into an empty interval and prevents Follow/Stop
    // problems from being assembled at all.  Acceleration and steering-rate
    // remain inset below because those are executable physical inputs.
    problem.state_weight.segment<kLegacyStateDimension>(state_offset) =
      request.states[static_cast<std::size_t>(stage)].weight;
    problem.additional_linear_cost.segment<kLegacyStateDimension>(state_offset) =
      request.states[static_cast<std::size_t>(stage)].linear_cost;

    const int source_input = std::max(0, stage - 1);
    const double requested_steering_reference = stage == 0 ?
      request.current_steering_rad :
      steering_from_curvature(
      request.wheelbase_m, request.curvature_reference_gain,
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
      steering_from_curvature(
        request.wheelbase_m, request.curvature_reference_gain, curvature_lower));
    const double steering_upper = std::min(
      request.maximum_abs_steering_rad,
      steering_from_curvature(
        request.wheelbase_m, request.curvature_reference_gain, curvature_upper));
    if (
      !std::isfinite(requested_steering_reference) ||
      !std::isfinite(steering_lower) || !std::isfinite(steering_upper) ||
      steering_lower > steering_upper)
    {
      return reject(
        RejectReason::SteeringBoundsUnavailable, stage,
        model::kSteeringIndex, requested_steering_reference, steering_lower,
        steering_upper);
    }
    // Curvature is a soft legacy reference, while steering is the physical
    // state owned by the seven-state model.  A reference curvature outside
    // the actuator envelope must therefore project to the nearest feasible
    // steering reference.  Rejecting the complete problem here creates an
    // authority hole exactly on the high-curvature stages where saturation is
    // expected.  This is reference normalization before optimization, not a
    // post-solve command clamp; the optimized state and rate prefix remain
    // certified against the unchanged physical bounds below.
    const double steering_reference = std::clamp(
      requested_steering_reference, steering_lower, steering_upper);
    problem.state_reference[state_offset + model::kSteeringIndex] =
      steering_reference;
    problem.state_lower[state_offset + model::kSteeringIndex] = steering_lower;
    problem.state_upper[state_offset + model::kSteeringIndex] = steering_upper;
    if (stage > 0) {
      const double previous_response =
        response_steering_reference_rad[static_cast<std::size_t>(stage - 1)];
      const double previous_steering =
        result.steering_reference_rad[static_cast<std::size_t>(stage - 1)];
      const double previous_dt =
        request.inputs[static_cast<std::size_t>(stage - 1)].stage_dt_sec;
      const auto previous_stage = static_cast<std::size_t>(stage - 1);
      const auto auxiliary = mpcc_vehicle_model::advance(
        mpcc_vehicle_model::State{0, 0, 0,
          problem.state_reference[(stage - 1) * model::kStateDimension + model::kVelocityIndex],
          lateral_velocity_reference_mps[previous_stage], yaw_rate_reference_radps[previous_stage],
          previous_steering, previous_response},
        mpcc_vehicle_model::Input{
          request.inputs[previous_stage].reference[model::kAccelerationIndex],
          (steering_reference - previous_steering) / previous_dt},
        request.vehicle_model, previous_dt);
      if (!auxiliary) {
        return reject(RejectReason::LinearizationUnavailable, stage);
      }
      response_steering_reference_rad[static_cast<std::size_t>(stage)] =
        auxiliary->state.tire_steering_rad;
      lateral_velocity_reference_mps[static_cast<std::size_t>(stage)] =
        auxiliary->state.lateral_velocity_mps;
      yaw_rate_reference_radps[static_cast<std::size_t>(stage)] =
        auxiliary->state.yaw_rate_radps;
    }
    const double response_steering_reference =
      response_steering_reference_rad[static_cast<std::size_t>(stage)];
    if (
      !std::isfinite(response_steering_reference) ||
      std::abs(response_steering_reference) >
      request.vehicle_model.maximum_wire_steering_rad * request.vehicle_model.tire_grip)
    {
      return reject(
        RejectReason::SteeringBoundsUnavailable, stage,
        model::kResponseSteeringIndex, response_steering_reference,
        -request.maximum_abs_steering_rad,
        request.maximum_abs_steering_rad);
    }
    problem.state_reference[state_offset + model::kResponseSteeringIndex] =
      response_steering_reference;
    problem.state_lower[state_offset + model::kResponseSteeringIndex] =
      -request.vehicle_model.maximum_wire_steering_rad * request.vehicle_model.tire_grip;
    problem.state_upper[state_offset + model::kResponseSteeringIndex] =
      request.vehicle_model.maximum_wire_steering_rad * request.vehicle_model.tire_grip;
    problem.state_weight[state_offset + model::kResponseSteeringIndex] = 0.0;
    problem.state_reference[state_offset + model::kLateralVelocityIndex] =
      lateral_velocity_reference_mps[static_cast<std::size_t>(stage)];
    problem.state_reference[state_offset + model::kYawRateIndex] =
      yaw_rate_reference_radps[static_cast<std::size_t>(stage)];
    // These are physical dynamic states, not new command channels or tuned
    // tracking objectives. The dynamics determine them; the existing exact
    // swept-body constraints certify their geometric effect.
    for (const int element : {model::kLateralVelocityIndex, model::kYawRateIndex}) {
      problem.state_lower[state_offset + element] = stage == 0 ?
        problem.initial_state[element] : -std::numeric_limits<double>::infinity();
      problem.state_upper[state_offset + element] = stage == 0 ?
        problem.initial_state[element] : std::numeric_limits<double>::infinity();
    }
    if (stage == horizon &&
      request.states.back().lower[model::kVelocityIndex] == 0.0 &&
      request.states.back().upper[model::kVelocityIndex] == 0.0) {
      for (const int element : {model::kLateralVelocityIndex, model::kYawRateIndex}) {
        problem.state_lower[state_offset + element] = 0.0;
        problem.state_upper[state_offset + element] = 0.0;
      }
    }
    result.steering_reference_rad[static_cast<std::size_t>(stage)] =
      steering_reference;
    result.steering_lower_rad[static_cast<std::size_t>(stage)] = steering_lower;
    result.steering_upper_rad[static_cast<std::size_t>(stage)] = steering_upper;
    if (stage > 0) {
      const auto & input = request.inputs[static_cast<std::size_t>(source_input)];
      const double jacobian = curvature_jacobian(
        request.wheelbase_m, request.curvature_reference_gain, steering_reference);
      if (!std::isfinite(jacobian) || jacobian <= 0.0) {
        return reject(
          RejectReason::SteeringJacobianUnavailable, stage,
          model::kSteeringIndex, jacobian);
      }
      problem.state_weight[state_offset + model::kSteeringIndex] =
        input.weight[kLegacyCurvatureIndex] * jacobian * jacobian;
    }
  }

  const bool braking_feasibility_valid = !request.maximum_braking_feasibility ||
    is_braking_feasibility_request(request);
  for (int stage = 0; stage < horizon; ++stage) {
    const auto index = static_cast<std::size_t>(stage);
    const auto & legacy_input = request.inputs[index];
    const int input_offset = model::kInputDimension * stage;
    const double steering_reference =
      result.steering_reference_rad[index];
    problem.input_reference[input_offset + model::kAccelerationIndex] =
      legacy_input.reference[0];
    problem.input_reference[input_offset + model::kSteeringRateIndex] = 0.0;
    problem.input_reference[
      input_offset + model::kVirtualProgressSpeedIndex] =
      legacy_input.reference[2];
    const auto acceleration_bounds = resolve_exact_physical_boundary_bounds(
      legacy_input.lower[0], legacy_input.upper[0], solver_tolerance);
    const double physical_rate_lower = stage == 0 ? std::max(
        -request.maximum_abs_steering_rate_radps,
        (-request.maximum_abs_steering_rad - request.current_steering_rad) /
        legacy_input.stage_dt_sec) : -request.maximum_abs_steering_rate_radps;
    const double physical_rate_upper = stage == 0 ? std::min(
        request.maximum_abs_steering_rate_radps,
        (request.maximum_abs_steering_rad - request.current_steering_rad) /
        legacy_input.stage_dt_sec) : request.maximum_abs_steering_rate_radps;
    const auto steering_rate_bounds = resolve_exact_physical_boundary_bounds(
      physical_rate_lower, physical_rate_upper, solver_tolerance);
    if (!acceleration_bounds.has_value()) {
      return reject(
        RejectReason::AccelerationInsetUnavailable, stage,
        model::kAccelerationIndex, legacy_input.reference[0],
        legacy_input.lower[0], legacy_input.upper[0]);
    }
    if (!steering_rate_bounds.has_value()) {
      return reject(
        RejectReason::SteeringRateInsetUnavailable, stage,
        model::kSteeringRateIndex, 0.0,
        physical_rate_lower, physical_rate_upper);
    }
    problem.input_lower[input_offset + model::kAccelerationIndex] =
      acceleration_bounds->lower;
    problem.input_upper[input_offset + model::kAccelerationIndex] =
      acceleration_bounds->upper;
    if (request.maximum_braking_feasibility) {
      if (!braking_feasibility_valid ||
        std::abs(legacy_input.reference[0] - acceleration_bounds->lower) > 1e-9)
      {
        return reject(RejectReason::AccelerationInsetUnavailable, stage);
      }
      problem.input_lower[input_offset + model::kAccelerationIndex] = acceleration_bounds->lower;
      problem.input_upper[input_offset + model::kAccelerationIndex] = acceleration_bounds->lower;
    }
    problem.input_lower[input_offset + model::kSteeringRateIndex] =
      steering_rate_bounds->lower;
    problem.input_upper[input_offset + model::kSteeringRateIndex] =
      steering_rate_bounds->upper;
    if (stage == 0) {
      result.first_steering_rate_physical_lower_radps = physical_rate_lower;
      result.first_steering_rate_physical_upper_radps = physical_rate_upper;
      result.first_steering_rate_solver_lower_radps = steering_rate_bounds->lower;
      result.first_steering_rate_solver_upper_radps = steering_rate_bounds->upper;
      result.first_steering_rate_certificate_margin_radps =
      steering_rate_bounds->certificate_margin;
    }
    // Virtual progress speed is an internal contouring state transition, not
    // a command crossing the publisher boundary.  A valid hold stage may
    // therefore be the singleton [0, 0], just like future velocity above.
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
      request.wheelbase_m, request.curvature_reference_gain, steering_reference);
    result.curvature_to_steering_jacobian_radpm_per_rad[index] = jacobian;
    const double curvature_change_weight =
      request.input_delta_weight[kLegacyCurvatureIndex];
    problem.input_weight[input_offset + model::kSteeringRateIndex] =
      curvature_change_weight * jacobian * jacobian *
      legacy_input.stage_dt_sec * legacy_input.stage_dt_sec;
  }
  // A soft racing reference can jump from an initial low speed to the target
  // speed in one stage. Such disconnected states are not a physical initial
  // trajectory and can make the first affine QP infeasible before refinement.
  // Keep every objective and bound above, but build all initial tangents along
  // one native trajectory. This seed has no execution or certificate authority.
  auto tangent_state = problem.initial_state;
  problem.linearizations.reserve(static_cast<std::size_t>(horizon));
  for (int stage = 0; stage < horizon; ++stage) {
    const auto & semantic_input = request.inputs[static_cast<std::size_t>(stage)];
    const int input = stage * model::kInputDimension;
    const double dt = semantic_input.stage_dt_sec;
    const double acceleration = std::clamp(
      problem.input_reference[input + model::kAccelerationIndex],
      problem.input_lower[input + model::kAccelerationIndex],
      problem.input_upper[input + model::kAccelerationIndex]);
    const auto & prefix = *problem.steering_rate_prefix_bounds;
    const double rate_lower = std::max(
      problem.input_lower[input + model::kSteeringRateIndex],
      (prefix.minimum_cumulative_delta_rad + request.current_steering_rad -
      tangent_state[model::kSteeringIndex]) / dt);
    const double rate_upper = std::min(
      problem.input_upper[input + model::kSteeringRateIndex],
      (prefix.maximum_cumulative_delta_rad + request.current_steering_rad -
      tangent_state[model::kSteeringIndex]) / dt);
    if (rate_lower > rate_upper) {
      return reject(RejectReason::LinearizationUnavailable, stage);
    }
    const double steering_rate = std::clamp(0.0, rate_lower, rate_upper);
    const double projected_speed =
      tangent_state[model::kVelocityIndex] * std::cos(tangent_state[model::kHeadingIndex]) -
      tangent_state[model::kLateralVelocityIndex] * std::sin(tangent_state[model::kHeadingIndex]);
    const double progress_lower = problem.input_lower[input + model::kVirtualProgressSpeedIndex];
    const double progress_upper = problem.input_upper[input + model::kVirtualProgressSpeedIndex];
    const auto virtual_speed = select_virtual_speed_tangent(
      request.course_frame, tangent_state[model::kProgressIndex], dt,
      std::clamp(projected_speed, progress_lower, progress_upper), progress_lower, progress_upper);
    if (!virtual_speed) {
      return reject(RejectReason::LinearizationUnavailable, stage);
    }
    const model::LinearizationRequest tangent_request{
      tangent_state[0], tangent_state[1], tangent_state[2], tangent_state[3],
      tangent_state[4], tangent_state[5], tangent_state[6], acceleration,
      steering_rate, *virtual_speed, semantic_input.path_curvature_radpm,
      request.wheelbase_m, dt, request.minimum_frenet_denominator,
      request.minimum_stage_dt_sec, request.maximum_stage_dt_sec, request.course_frame,
      tangent_state[7], tangent_state[8], request.vehicle_model};
    const auto next = model::evaluate_temporal_frenet_transition(tangent_request);
    const auto linearization = model::linearize_temporal_frenet(tangent_request);
    if (!next || !linearization) {
      return reject(RejectReason::LinearizationUnavailable, stage);
    }
    problem.linearizations.push_back(*linearization);
    tangent_state = next->next_state;
  }
  return result;
}

RelinearizationResult relinearize_around_primal(
  const Request & request, const Eigen::VectorXd & primal,
  mpcc_rate_resolved_problem::AssemblyRequest & problem) noexcept
{
  namespace model = mpcc_rate_resolved;
  const int horizon = request.horizon_steps;
  const int state_values = model::kStateDimension * (horizon + 1);
  const int variable_count = state_values + model::kInputDimension * horizon;
  RelinearizationResult result;
  if (
    horizon <= 0 || request.inputs.size() != static_cast<std::size_t>(horizon) ||
    problem.horizon_steps != horizon ||
    problem.linearizations.size() != static_cast<std::size_t>(horizon))
  {
    return result;
  }
  if (primal.size() != variable_count || !primal.allFinite()) {
    result.reason = RelinearizationReason::InvalidPrimal;
    return result;
  }

  std::vector<model::Linearization> linearizations;
  linearizations.reserve(static_cast<std::size_t>(horizon));
  for (int stage = 0; stage < horizon; ++stage) {
    const int input = state_values + model::kInputDimension * stage;
    const int problem_input = model::kInputDimension * stage;
    const auto & semantic_input =
      request.inputs[static_cast<std::size_t>(stage)];
    const auto selected_state = mpcc_rate_resolved_problem::select_linearization_state(
      problem, primal, stage);
    if (!selected_state) {
      result.reason = RelinearizationReason::InvalidPrimal;
      result.stage = stage;
      return result;
    }
    const auto & linearization_state = *selected_state;
    Eigen::Matrix<double, model::kInputDimension, 1> linearization_input;
    for (int element = 0; element < model::kInputDimension; ++element) {
      linearization_input[element] = std::clamp(
        primal[input + element],
        problem.input_lower[problem_input + element],
        problem.input_upper[problem_input + element]);
    }
    const auto virtual_speed_tangent = select_virtual_speed_tangent(
      request.course_frame, linearization_state[model::kProgressIndex],
      semantic_input.stage_dt_sec, linearization_input[model::kVirtualProgressSpeedIndex],
      problem.input_lower[problem_input + model::kVirtualProgressSpeedIndex],
      problem.input_upper[problem_input + model::kVirtualProgressSpeedIndex]);
    if (!virtual_speed_tangent) {
      result.reason = RelinearizationReason::LinearizationUnavailable;
      result.stage = stage;
      return result;
    }
    linearization_input[model::kVirtualProgressSpeedIndex] = *virtual_speed_tangent;
    const auto linearization = model::linearize_temporal_frenet(
      model::LinearizationRequest{
        linearization_state[model::kLateralIndex],
        linearization_state[model::kLagIndex],
        linearization_state[model::kHeadingIndex],
        linearization_state[model::kVelocityIndex],
        linearization_state[model::kProgressIndex],
        linearization_state[model::kSteeringIndex],
        linearization_state[model::kResponseSteeringIndex],
        linearization_input[model::kAccelerationIndex],
        linearization_input[model::kSteeringRateIndex],
        linearization_input[model::kVirtualProgressSpeedIndex],
        semantic_input.path_curvature_radpm,
        request.wheelbase_m,
        semantic_input.stage_dt_sec,
        request.minimum_frenet_denominator,
        request.minimum_stage_dt_sec,
        request.maximum_stage_dt_sec, request.course_frame,
        linearization_state[model::kLateralVelocityIndex],
        linearization_state[model::kYawRateIndex], request.vehicle_model});
    if (!linearization.has_value()) {
      result.reason = RelinearizationReason::LinearizationUnavailable;
      result.stage = stage;
      return result;
    }
    linearizations.push_back(linearization.value());
  }
  problem.linearizations = std::move(linearizations);
  result.reason = RelinearizationReason::Accepted;
  result.applied = true;
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_adapter
