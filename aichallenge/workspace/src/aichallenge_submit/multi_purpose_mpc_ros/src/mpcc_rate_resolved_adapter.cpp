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
  const double wheelbase_m, const double yaw_response_gain,
  const double curvature_radpm) noexcept
{
  // The seven-state model defines the steady yaw response as
  //   kappa = yaw_response_gain * tan(delta_response) / wheelbase.
  // Curvature references and bounds must use that same contract.  Reusing
  // the unit-gain bicycle conversion under-steers every future reference
  // whenever the identified response gain is below one, leaving feedback to
  // discover the deficit only after a heading error has already developed.
  return std::atan(wheelbase_m * curvature_radpm / yaw_response_gain);
}

double curvature_jacobian(
  const double wheelbase_m, const double yaw_response_gain,
  const double steering_rad) noexcept
{
  const double cosine = std::cos(steering_rad);
  return yaw_response_gain / (wheelbase_m * cosine * cosine);
}

struct SolverInsetBounds
{
  double lower{};
  double upper{};
  double margin{};
};

/// OSQP's certified solution may lie outside a solver row by its accepted
/// residual.  A command/publisher boundary, however, must satisfy the physical
/// actuator envelope exactly.  Inset the solver row by the maximum certified
/// residual instead of clamping a solved command after certification.
std::optional<SolverInsetBounds> inset_for_exact_physical_boundary(
  const double physical_lower, const double physical_upper,
  const persistent_osqp::PhysicalConstraintTolerance & tolerance) noexcept
{
  if (
    std::isnan(physical_lower) || std::isnan(physical_upper) ||
    physical_lower > physical_upper || !std::isfinite(tolerance.absolute) ||
    tolerance.absolute < 0.0 || !std::isfinite(tolerance.relative) ||
    tolerance.relative < 0.0 || tolerance.relative >= 1.0)
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
    (tolerance.absolute + tolerance.relative * characteristic) /
    (1.0 - tolerance.relative);
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
  return SolverInsetBounds{solver_lower, solver_upper, margin};
}

}  // namespace

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
    !std::isfinite(request.wheelbase_m) || request.wheelbase_m <= 0.0 ||
    !std::isfinite(request.yaw_response_gain) ||
    request.yaw_response_gain <= 0.0 ||
    !std::isfinite(request.yaw_response_time_constant_sec) ||
    request.yaw_response_time_constant_sec <= 0.0 ||
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
  for (int element = 0; element < kLegacyStateDimension; ++element) {
    if (
      request.initial_state[element] < request.states.front().lower[element] ||
      request.initial_state[element] > request.states.front().upper[element])
    {
      return reject(
        RejectReason::InitialStateOutsideBounds, 0, element,
        request.initial_state[element], request.states.front().lower[element],
        request.states.front().upper[element]);
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
  result.curvature_to_steering_jacobian_radpm_per_rad.resize(
    static_cast<std::size_t>(horizon));
  // The physical steering state is the only origin of the optimized rate
  // prefix. Integrating that rate again from the previous desired command
  // creates a permanently offset trajectory outside the QP wall proof.
  const double physical_prefix_lower =
    -request.maximum_abs_steering_rad - request.current_steering_rad;
  const double physical_prefix_upper =
    request.maximum_abs_steering_rad - request.current_steering_rad;
  const auto solver_prefix_bounds = inset_for_exact_physical_boundary(
    physical_prefix_lower, physical_prefix_upper, solver_tolerance);
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
      request.wheelbase_m, request.yaw_response_gain,
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
        request.wheelbase_m, request.yaw_response_gain, curvature_lower));
    const double steering_upper = std::min(
      request.maximum_abs_steering_rad,
      steering_from_curvature(
        request.wheelbase_m, request.yaw_response_gain, curvature_upper));
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
      const double response_decay = std::exp(
        -previous_dt / request.yaw_response_time_constant_sec);
      response_steering_reference_rad[static_cast<std::size_t>(stage)] =
        previous_steering +
        (previous_response - previous_steering) * response_decay;
    }
    const double response_steering_reference =
      response_steering_reference_rad[static_cast<std::size_t>(stage)];
    if (
      !std::isfinite(response_steering_reference) ||
      std::abs(response_steering_reference) >
      request.maximum_abs_steering_rad)
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
      -request.maximum_abs_steering_rad;
    problem.state_upper[state_offset + model::kResponseSteeringIndex] =
      request.maximum_abs_steering_rad;
    problem.state_weight[state_offset + model::kResponseSteeringIndex] = 0.0;
    result.steering_reference_rad[static_cast<std::size_t>(stage)] =
      steering_reference;
    result.steering_lower_rad[static_cast<std::size_t>(stage)] = steering_lower;
    result.steering_upper_rad[static_cast<std::size_t>(stage)] = steering_upper;
    if (stage > 0) {
      const auto & input = request.inputs[static_cast<std::size_t>(source_input)];
      const double jacobian = curvature_jacobian(
        request.wheelbase_m, request.yaw_response_gain, steering_reference);
      if (!std::isfinite(jacobian) || jacobian <= 0.0) {
        return reject(
          RejectReason::SteeringJacobianUnavailable, stage,
          model::kSteeringIndex, jacobian);
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
        response_steering_reference_rad[index],
        legacy_input.reference[0], 0.0, legacy_input.reference[2],
        legacy_input.path_curvature_radpm, request.wheelbase_m,
        request.yaw_response_gain,
        request.yaw_response_time_constant_sec,
        legacy_input.stage_dt_sec, request.minimum_frenet_denominator,
        request.minimum_stage_dt_sec, request.maximum_stage_dt_sec});
    if (!linearization.has_value()) {
      return reject(RejectReason::LinearizationUnavailable, stage);
    }
    problem.linearizations.push_back(linearization.value());

    problem.input_reference[input_offset + model::kAccelerationIndex] =
      legacy_input.reference[0];
    problem.input_reference[input_offset + model::kSteeringRateIndex] = 0.0;
    problem.input_reference[
      input_offset + model::kVirtualProgressSpeedIndex] =
      legacy_input.reference[2];
    const auto acceleration_bounds = inset_for_exact_physical_boundary(
      legacy_input.lower[0], legacy_input.upper[0], solver_tolerance);
    const double physical_rate_lower = stage == 0 ? std::max(
        -request.maximum_abs_steering_rate_radps,
        (-request.maximum_abs_steering_rad - request.current_steering_rad) /
        legacy_input.stage_dt_sec) : -request.maximum_abs_steering_rate_radps;
    const double physical_rate_upper = stage == 0 ? std::min(
        request.maximum_abs_steering_rate_radps,
        (request.maximum_abs_steering_rad - request.current_steering_rad) /
        legacy_input.stage_dt_sec) : request.maximum_abs_steering_rate_radps;
    const auto steering_rate_bounds = inset_for_exact_physical_boundary(
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
        steering_rate_bounds->margin;
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
      request.wheelbase_m, request.yaw_response_gain, steering_reference);
    result.curvature_to_steering_jacobian_radpm_per_rad[index] = jacobian;
    const double curvature_change_weight =
      request.input_delta_weight[kLegacyCurvatureIndex];
    problem.input_weight[input_offset + model::kSteeringRateIndex] =
      curvature_change_weight * jacobian * jacobian *
      legacy_input.stage_dt_sec * legacy_input.stage_dt_sec;
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
    const int state = model::kStateDimension * stage;
    const int input = state_values + model::kInputDimension * stage;
    const auto & semantic_input =
      request.inputs[static_cast<std::size_t>(stage)];
    const auto linearization = model::linearize_temporal_frenet(
      model::LinearizationRequest{
        primal[state + model::kLateralIndex],
        primal[state + model::kLagIndex],
        primal[state + model::kHeadingIndex],
        primal[state + model::kVelocityIndex],
        primal[state + model::kProgressIndex],
        primal[state + model::kSteeringIndex],
        primal[state + model::kResponseSteeringIndex],
        primal[input + model::kAccelerationIndex],
        primal[input + model::kSteeringRateIndex],
        primal[input + model::kVirtualProgressSpeedIndex],
        semantic_input.path_curvature_radpm,
        request.wheelbase_m,
        request.yaw_response_gain,
        request.yaw_response_time_constant_sec,
        semantic_input.stage_dt_sec,
        request.minimum_frenet_denominator,
        request.minimum_stage_dt_sec,
        request.maximum_stage_dt_sec});
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
