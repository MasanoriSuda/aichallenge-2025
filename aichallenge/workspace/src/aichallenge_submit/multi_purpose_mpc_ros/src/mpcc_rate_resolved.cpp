#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"

#include <algorithm>
#include <cmath>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved
{
namespace
{

constexpr double kNumericalDerivativeRelativeStep = 1.0e-6;

bool transition_request_valid(const LinearizationRequest & request) noexcept
{
  constexpr double half_pi = 1.57079632679489661923;
  return
    std::isfinite(request.reference_lateral_m) &&
    std::isfinite(request.reference_lag_m) &&
    std::isfinite(request.reference_heading_rad) &&
    std::isfinite(request.reference_velocity_mps) &&
    std::isfinite(request.reference_progress_m) &&
    std::isfinite(request.course_frame.progress_origin_m) &&
    std::isfinite(request.reference_steering_rad) &&
    std::abs(request.reference_steering_rad) < half_pi &&
    std::isfinite(request.reference_response_steering_rad) &&
    std::abs(request.reference_response_steering_rad) < half_pi &&
    std::isfinite(request.reference_acceleration_mps2) &&
    std::isfinite(request.reference_steering_rate_radps) &&
    std::isfinite(request.reference_virtual_progress_speed_mps) &&
    std::isfinite(request.reference_path_curvature_radpm) &&
    std::isfinite(request.wheelbase_m) && request.wheelbase_m > 0.0 &&
    std::isfinite(request.yaw_response_gain) &&
    request.yaw_response_gain > 0.0 &&
    std::isfinite(request.yaw_response_time_constant_sec) &&
    request.yaw_response_time_constant_sec > 0.0 &&
    std::isfinite(request.stage_dt_sec) &&
    std::isfinite(request.minimum_frenet_denominator) &&
    request.minimum_frenet_denominator > 0.0 &&
    std::isfinite(request.minimum_stage_dt_sec) &&
    request.minimum_stage_dt_sec > 0.0 &&
    std::isfinite(request.maximum_stage_dt_sec) &&
    request.maximum_stage_dt_sec >= request.minimum_stage_dt_sec &&
    request.stage_dt_sec >= request.minimum_stage_dt_sec &&
    request.stage_dt_sec <= request.maximum_stage_dt_sec;
}

StateVector request_state(const LinearizationRequest & request) noexcept
{
  return (StateVector() <<
    request.reference_lateral_m, request.reference_lag_m,
    request.reference_heading_rad, request.reference_velocity_mps,
    request.reference_progress_m, request.reference_steering_rad,
    request.reference_response_steering_rad).finished();
}

InputVector request_input(const LinearizationRequest & request) noexcept
{
  return (InputVector() <<
    request.reference_acceleration_mps2,
    request.reference_steering_rate_radps,
    request.reference_virtual_progress_speed_mps).finished();
}

void set_request_state(
  LinearizationRequest & request, const StateVector & state) noexcept
{
  request.reference_lateral_m = state[kLateralIndex];
  request.reference_lag_m = state[kLagIndex];
  request.reference_heading_rad = state[kHeadingIndex];
  request.reference_velocity_mps = state[kVelocityIndex];
  request.reference_progress_m = state[kProgressIndex];
  request.reference_steering_rad = state[kSteeringIndex];
  request.reference_response_steering_rad = state[kResponseSteeringIndex];
}

void set_request_input(
  LinearizationRequest & request, const InputVector & input) noexcept
{
  request.reference_acceleration_mps2 = input[kAccelerationIndex];
  request.reference_steering_rate_radps = input[kSteeringRateIndex];
  request.reference_virtual_progress_speed_mps =
    input[kVirtualProgressSpeedIndex];
}

double response_steering_after_ramp(
  const double initial_command_rad,
  const double initial_response_rad,
  const double steering_rate_radps,
  const double elapsed_sec,
  const double time_constant_sec) noexcept
{
  const double decay = std::exp(-elapsed_sec / time_constant_sec);
  return initial_command_rad +
         (initial_response_rad - initial_command_rad) * decay +
         steering_rate_radps *
         (elapsed_sec - time_constant_sec * (1.0 - decay));
}

}  // namespace

bool course_frame_matches(
  const CourseFrame & frame,
  const std::vector<mpc_stage_geometry::CourseFrameKnot> & knots,
  const double progress_origin_m) noexcept
{
  return frame.knots && knots.size() >= 2U &&
         frame.progress_origin_m == progress_origin_m &&
         frame.knots->size() == knots.size() &&
         std::equal(frame.knots->begin(), frame.knots->end(), knots.begin(),
    [](const auto & lhs, const auto & rhs) {
      return lhs.progress_m == rhs.progress_m && lhs.x_m == rhs.x_m &&
             lhs.y_m == rhs.y_m && lhs.heading_rad == rhs.heading_rad &&
             lhs.waypoint == rhs.waypoint;
    });
}

std::optional<NonlinearTransition> evaluate_temporal_frenet_transition(
  const LinearizationRequest & request) noexcept
{
  if (!transition_request_valid(request)) {
    return std::nullopt;
  }
  const auto substep_count = static_cast<std::size_t>(std::max(
      1.0, std::ceil(
        request.stage_dt_sec / kMaximumPhysicalIntegrationStepSec)));
  const double step_sec =
    request.stage_dt_sec / static_cast<double>(substep_count);
  StateVector state = request_state(request);
  const InputVector input = request_input(request);
  // Integrate in a fixed physical frame whose origin/orientation are the
  // initial course pose. Keeping coordinates local avoids subtractive noise
  // in finite-difference Jacobians at large MGRS world coordinates.
  double x_m = state[kLagIndex];
  double y_m = state[kLateralIndex];
  double yaw_rad = state[kHeadingIndex];
  const double progress_delta_m =
    input[kVirtualProgressSpeedIndex] * request.stage_dt_sec;
  double frame_dx_m{};
  double frame_dy_m{};
  double frame_heading_delta_rad{};
  if (request.course_frame.knots) {
    const auto & knots = *request.course_frame.knots;
    const double initial_progress_m =
      request.course_frame.progress_origin_m + state[kProgressIndex];
    const auto initial = mpc_stage_geometry::sample_course_frame(
      knots, initial_progress_m, 1e-9);
    const auto terminal = mpc_stage_geometry::sample_course_frame(
      knots, initial_progress_m + progress_delta_m, 1e-9);
    if (!initial || !terminal) {
      return std::nullopt;
    }
    const double dx = terminal->x_m - initial->x_m;
    const double dy = terminal->y_m - initial->y_m;
    const double c = std::cos(initial->heading_rad);
    const double s = std::sin(initial->heading_rad);
    frame_dx_m = c * dx + s * dy;
    frame_dy_m = -s * dx + c * dy;
    frame_heading_delta_rad = std::atan2(
      std::sin(terminal->heading_rad - initial->heading_rad),
      std::cos(terminal->heading_rad - initial->heading_rad));
  } else {
    // Exact arc-length geometry, not a differential approximation to a
    // separately interpolated world course.
    frame_heading_delta_rad =
      request.reference_path_curvature_radpm * progress_delta_m;
    const double half_angle = 0.5 * frame_heading_delta_rad;
    const double sinc = std::abs(half_angle) < 1e-8 ?
      1.0 - half_angle * half_angle / 6.0 : std::sin(half_angle) / half_angle;
    frame_dx_m = progress_delta_m * sinc * std::cos(half_angle);
    frame_dy_m = progress_delta_m * sinc * std::sin(half_angle);
  }
  // Preserve the existing local-chart validity bound; it no longer divides
  // physical velocity or introduces fictitious motion into lag/lateral state.
  if (1.0 - request.reference_path_curvature_radpm * state[kLateralIndex] <
    request.minimum_frenet_denominator)
  {
    return std::nullopt;
  }
  for (std::size_t substep = 0U; substep < substep_count; ++substep) {
    const double steering = state[kSteeringIndex];
    const double response = state[kResponseSteeringIndex];
    const double steering_rate = input[kSteeringRateIndex];
    const double response_mid = response_steering_after_ramp(
      steering, response, steering_rate, 0.5 * step_sec,
      request.yaw_response_time_constant_sec);
    const double response_next = response_steering_after_ramp(
      steering, response, steering_rate, step_sec,
      request.yaw_response_time_constant_sec);
    const double velocity_mid = state[kVelocityIndex] +
      0.5 * input[kAccelerationIndex] * step_sec;
    const double yaw_rate_mid =
      request.yaw_response_gain * velocity_mid * std::tan(response_mid) /
      request.wheelbase_m;
    const double yaw_mid = yaw_rad + 0.5 * yaw_rate_mid * step_sec;
    if (
      !std::isfinite(response_mid) || !std::isfinite(response_next) ||
      !std::isfinite(velocity_mid) || !std::isfinite(yaw_mid) ||
      !std::isfinite(yaw_rate_mid))
    {
      return std::nullopt;
    }
    x_m += velocity_mid * std::cos(yaw_mid) * step_sec;
    y_m += velocity_mid * std::sin(yaw_mid) * step_sec;
    yaw_rad += yaw_rate_mid * step_sec;
    state[kVelocityIndex] += input[kAccelerationIndex] * step_sec;
    state[kProgressIndex] +=
      input[kVirtualProgressSpeedIndex] * step_sec;
    state[kSteeringIndex] += steering_rate * step_sec;
    state[kResponseSteeringIndex] = response_next;
    if (!state.allFinite()) {
      return std::nullopt;
    }
  }
  const double dx = x_m - frame_dx_m;
  const double dy = y_m - frame_dy_m;
  const double c = std::cos(frame_heading_delta_rad);
  const double s = std::sin(frame_heading_delta_rad);
  state[kLagIndex] = c * dx + s * dy;
  state[kLateralIndex] = -s * dx + c * dy;
  state[kHeadingIndex] = std::atan2(
    std::sin(yaw_rad - frame_heading_delta_rad),
    std::cos(yaw_rad - frame_heading_delta_rad));
  if (!state.allFinite() ||
    1.0 - request.reference_path_curvature_radpm * state[kLateralIndex] <
    request.minimum_frenet_denominator)
  {
    return std::nullopt;
  }
  return NonlinearTransition{state, substep_count};
}

std::optional<Linearization> linearize_temporal_frenet(
  const LinearizationRequest & request) noexcept
{
  if (
    request.reference_velocity_mps < 0.0 ||
    request.reference_virtual_progress_speed_mps < 0.0)
  {
    return std::nullopt;
  }
  const auto reference_transition =
    evaluate_temporal_frenet_transition(request);
  if (!reference_transition.has_value()) {
    return std::nullopt;
  }
  using StateMatrix = Eigen::Matrix<double, kStateDimension, kStateDimension>;
  using InputMatrix = Eigen::Matrix<double, kStateDimension, kInputDimension>;
  const StateVector reference_state = request_state(request);
  const InputVector reference_input = request_input(request);
  Linearization result;
  result.state_matrix = StateMatrix::Zero();
  result.input_matrix = InputMatrix::Zero();
  const auto numerical_column = [&](const bool state_column, const int element)
      -> std::optional<StateVector> {
      const double reference_value = state_column ?
        reference_state[element] : reference_input[element];
      const double delta = kNumericalDerivativeRelativeStep *
        std::max(1.0, std::abs(reference_value));
      auto plus_request = request;
      auto minus_request = request;
      if (state_column) {
        auto plus_state = reference_state;
        auto minus_state = reference_state;
        plus_state[element] += delta;
        minus_state[element] -= delta;
        set_request_state(plus_request, plus_state);
        set_request_state(minus_request, minus_state);
      } else {
        auto plus_input = reference_input;
        auto minus_input = reference_input;
        plus_input[element] += delta;
        minus_input[element] -= delta;
        set_request_input(plus_request, plus_input);
        set_request_input(minus_request, minus_input);
      }
      const auto plus = evaluate_temporal_frenet_transition(plus_request);
      const auto minus = evaluate_temporal_frenet_transition(minus_request);
      if (plus.has_value() && minus.has_value()) {
        return (plus->next_state - minus->next_state) / (2.0 * delta);
      }
      if (plus.has_value()) {
        return (plus->next_state - reference_transition->next_state) / delta;
      }
      if (minus.has_value()) {
        return (reference_transition->next_state - minus->next_state) / delta;
      }
      return std::nullopt;
    };
  for (int element = 0; element < kStateDimension; ++element) {
    const auto column = numerical_column(true, element);
    if (!column.has_value()) {
      return std::nullopt;
    }
    result.state_matrix.col(element) = column.value();
  }
  for (int element = 0; element < kInputDimension; ++element) {
    const auto column = numerical_column(false, element);
    if (!column.has_value()) {
      return std::nullopt;
    }
    result.input_matrix.col(element) = column.value();
  }

  // These rows are exactly affine under the stage's constant inputs.  Keep
  // their solver rows bit-for-bit consistent with the immutable execution
  // artifact contract instead of retaining finite-difference noise.
  result.state_matrix.row(kVelocityIndex).setZero();
  result.input_matrix.row(kVelocityIndex).setZero();
  result.state_matrix(kVelocityIndex, kVelocityIndex) = 1.0;
  result.input_matrix(kVelocityIndex, kAccelerationIndex) =
    request.stage_dt_sec;

  result.state_matrix.row(kProgressIndex).setZero();
  result.input_matrix.row(kProgressIndex).setZero();
  result.state_matrix(kProgressIndex, kProgressIndex) = 1.0;
  result.input_matrix(kProgressIndex, kVirtualProgressSpeedIndex) =
    request.stage_dt_sec;

  result.state_matrix.row(kSteeringIndex).setZero();
  result.input_matrix.row(kSteeringIndex).setZero();
  result.state_matrix(kSteeringIndex, kSteeringIndex) = 1.0;
  result.input_matrix(kSteeringIndex, kSteeringRateIndex) =
    request.stage_dt_sec;

  const double response_decay = std::exp(
    -request.stage_dt_sec / request.yaw_response_time_constant_sec);
  const double response_rate_integral_sec =
    request.stage_dt_sec - request.yaw_response_time_constant_sec *
    (1.0 - response_decay);
  result.state_matrix.row(kResponseSteeringIndex).setZero();
  result.input_matrix.row(kResponseSteeringIndex).setZero();
  result.state_matrix(kResponseSteeringIndex, kSteeringIndex) =
    1.0 - response_decay;
  result.state_matrix(kResponseSteeringIndex, kResponseSteeringIndex) =
    response_decay;
  result.input_matrix(kResponseSteeringIndex, kSteeringRateIndex) =
    response_rate_integral_sec;

  result.stage_dt_sec = request.stage_dt_sec;
  result.equality_offset =
    result.state_matrix * reference_state + result.input_matrix *
    reference_input - reference_transition->next_state;

  if (
    !result.state_matrix.allFinite() || !result.input_matrix.allFinite() ||
    !result.equality_offset.allFinite())
  {
    return std::nullopt;
  }
  return result;
}

const char * to_string(const ActuationSampleReason reason) noexcept
{
  switch (reason) {
    case ActuationSampleReason::Accepted:
      return "accepted";
    case ActuationSampleReason::InitialSteeringNonfinite:
      return "initial-steering-nonfinite";
    case ActuationSampleReason::SteeringRateNonfinite:
      return "steering-rate-nonfinite";
    case ActuationSampleReason::ElapsedTimeInvalid:
      return "elapsed-time-invalid";
    case ActuationSampleReason::StageDurationInvalid:
      return "stage-duration-invalid";
    case ActuationSampleReason::StageSequenceInvalid:
      return "stage-sequence-invalid";
    case ActuationSampleReason::PublicationAfterStageEnd:
      return "publication-after-stage-end";
    case ActuationSampleReason::PublicationAfterHorizonEnd:
      return "publication-after-horizon-end";
    case ActuationSampleReason::SteeringLimitInvalid:
      return "steering-limit-invalid";
    case ActuationSampleReason::SteeringRateLimitInvalid:
      return "steering-rate-limit-invalid";
    case ActuationSampleReason::WheelbaseInvalid:
      return "wheelbase-invalid";
    case ActuationSampleReason::InitialSteeringLimitViolation:
      return "initial-steering-limit-violation";
    case ActuationSampleReason::SteeringRateLimitViolation:
      return "steering-rate-limit-violation";
    case ActuationSampleReason::TerminalSteeringLimitViolation:
      return "terminal-steering-limit-violation";
    case ActuationSampleReason::SampledSteeringLimitViolation:
      return "sampled-steering-limit-violation";
    case ActuationSampleReason::CurvatureNonfinite:
      return "curvature-nonfinite";
    case ActuationSampleReason::SolverCertificateInvalid:
      return "solver-certificate-invalid";
    case ActuationSampleReason::Count:
      break;
  }
  return "unknown";
}

ActuationSampleEvaluation evaluate_actuation_sample(
  const ActuationSampleRequest & request) noexcept
{
  constexpr double half_pi = 1.57079632679489661923;
  constexpr double tolerance = 1e-12;
  ActuationSampleEvaluation result;
  if (!std::isfinite(request.initial_steering_rad)) {
    result.reason = ActuationSampleReason::InitialSteeringNonfinite;
    return result;
  }
  if (!std::isfinite(request.steering_rate_radps)) {
    result.reason = ActuationSampleReason::SteeringRateNonfinite;
    return result;
  }
  if (!std::isfinite(request.elapsed_sec) || request.elapsed_sec < 0.0) {
    result.reason = ActuationSampleReason::ElapsedTimeInvalid;
    return result;
  }
  if (
    !std::isfinite(request.stage_duration_sec) ||
    request.stage_duration_sec <= 0.0)
  {
    result.reason = ActuationSampleReason::StageDurationInvalid;
    return result;
  }
  if (request.elapsed_sec > request.stage_duration_sec + tolerance) {
    result.reason = ActuationSampleReason::PublicationAfterStageEnd;
    return result;
  }
  if (
    !std::isfinite(request.maximum_abs_steering_rad) ||
    request.maximum_abs_steering_rad <= 0.0 ||
    request.maximum_abs_steering_rad >= half_pi)
  {
    result.reason = ActuationSampleReason::SteeringLimitInvalid;
    return result;
  }
  if (
    !std::isfinite(request.maximum_abs_steering_rate_radps) ||
    request.maximum_abs_steering_rate_radps < 0.0)
  {
    result.reason = ActuationSampleReason::SteeringRateLimitInvalid;
    return result;
  }
  if (!std::isfinite(request.wheelbase_m) || request.wheelbase_m <= 0.0) {
    result.reason = ActuationSampleReason::WheelbaseInvalid;
    return result;
  }
  if (
    std::abs(request.initial_steering_rad) >
    request.maximum_abs_steering_rad + tolerance)
  {
    result.reason = ActuationSampleReason::InitialSteeringLimitViolation;
    return result;
  }
  if (
    std::abs(request.steering_rate_radps) >
    request.maximum_abs_steering_rate_radps + tolerance)
  {
    result.reason = ActuationSampleReason::SteeringRateLimitViolation;
    return result;
  }

  result.terminal_steering_rad =
    request.initial_steering_rad +
    request.steering_rate_radps * request.stage_duration_sec;
  result.sampled_steering_rad =
    request.initial_steering_rad +
    request.steering_rate_radps * request.elapsed_sec;
  if (
    std::abs(result.terminal_steering_rad) >
    request.maximum_abs_steering_rad + tolerance)
  {
    result.reason = ActuationSampleReason::TerminalSteeringLimitViolation;
    return result;
  }
  if (
    std::abs(result.sampled_steering_rad) >
    request.maximum_abs_steering_rad + tolerance)
  {
    result.reason = ActuationSampleReason::SampledSteeringLimitViolation;
    return result;
  }
  const double curvature =
    std::tan(result.sampled_steering_rad) / request.wheelbase_m;
  if (!std::isfinite(curvature)) {
    result.reason = ActuationSampleReason::CurvatureNonfinite;
    return result;
  }
  result.reason = ActuationSampleReason::Accepted;
  result.sample = ActuationSample{result.sampled_steering_rad, curvature};
  return result;
}

CertifiedActuationSequenceSampleEvaluation
evaluate_certified_actuation_sequence_sample(
  const CertifiedActuationSequenceSampleRequest & request) noexcept
{
  constexpr double half_pi = 1.57079632679489661923;
  constexpr double tolerance = 1e-12;
  CertifiedActuationSequenceSampleEvaluation result;
  if (!std::isfinite(request.semantic_initial_steering_rad)) {
    result.reason = ActuationSampleReason::InitialSteeringNonfinite;
    return result;
  }
  if (!std::isfinite(request.elapsed_sec) || request.elapsed_sec < 0.0) {
    result.reason = ActuationSampleReason::ElapsedTimeInvalid;
    return result;
  }
  if (
    request.certified_steering_rates_radps.empty() ||
    request.certified_steering_rates_radps.size() !=
    request.stage_durations_sec.size())
  {
    result.reason = ActuationSampleReason::StageSequenceInvalid;
    return result;
  }
  if (
    !std::isfinite(request.maximum_abs_steering_rad) ||
    request.maximum_abs_steering_rad <= 0.0 ||
    request.maximum_abs_steering_rad >= half_pi)
  {
    result.reason = ActuationSampleReason::SteeringLimitInvalid;
    return result;
  }
  if (!std::isfinite(request.wheelbase_m) || request.wheelbase_m <= 0.0) {
    result.reason = ActuationSampleReason::WheelbaseInvalid;
    return result;
  }
  if (
    !std::isfinite(request.maximum_normalized_constraint_violation) ||
    request.maximum_normalized_constraint_violation < 0.0 ||
    request.maximum_normalized_constraint_violation > 1.0)
  {
    result.reason = ActuationSampleReason::SolverCertificateInvalid;
    return result;
  }
  if (
    std::abs(request.semantic_initial_steering_rad) >
    request.maximum_abs_steering_rad + tolerance)
  {
    result.reason = ActuationSampleReason::InitialSteeringLimitViolation;
    return result;
  }

  for (
    std::size_t index = 0U; index < request.stage_durations_sec.size();
    ++index)
  {
    const double rate = request.certified_steering_rates_radps[index];
    const double duration = request.stage_durations_sec[index];
    if (!std::isfinite(rate)) {
      result.reason = ActuationSampleReason::SteeringRateNonfinite;
      return result;
    }
    if (!std::isfinite(duration) || duration <= 0.0) {
      result.reason = ActuationSampleReason::StageDurationInvalid;
      return result;
    }
    result.certified_horizon_duration_sec += duration;
    if (!std::isfinite(result.certified_horizon_duration_sec)) {
      result.reason = ActuationSampleReason::StageDurationInvalid;
      return result;
    }
  }
  if (
    request.elapsed_sec >
    result.certified_horizon_duration_sec + tolerance)
  {
    result.reason = ActuationSampleReason::PublicationAfterHorizonEnd;
    return result;
  }

  double remaining_sec = request.elapsed_sec;
  double steering_rad = request.semantic_initial_steering_rad;
  for (
    std::size_t index = 0U; index < request.stage_durations_sec.size();
    ++index)
  {
    const double duration = request.stage_durations_sec[index];
    const double stage_elapsed_sec = std::min(remaining_sec, duration);
    steering_rad +=
      request.certified_steering_rates_radps[index] * stage_elapsed_sec;
    if (
      !std::isfinite(steering_rad) ||
      std::abs(steering_rad) > request.maximum_abs_steering_rad + tolerance)
    {
      result.reason = ActuationSampleReason::SampledSteeringLimitViolation;
      result.sampled_steering_rad = steering_rad;
      return result;
    }
    if (remaining_sec <= duration + tolerance) {
      result.sampled_stage_index = index;
      result.sampled_stage_elapsed_sec = stage_elapsed_sec;
      break;
    }
    remaining_sec -= duration;
  }
  result.sampled_steering_rad = steering_rad;
  const double curvature =
    std::tan(result.sampled_steering_rad) / request.wheelbase_m;
  if (!std::isfinite(curvature)) {
    result.reason = ActuationSampleReason::CurvatureNonfinite;
    return result;
  }
  result.reason = ActuationSampleReason::Accepted;
  result.sample = ActuationSample{result.sampled_steering_rad, curvature};
  return result;
}

std::optional<ActuationSample> sample_actuation(
  const ActuationSampleRequest & request) noexcept
{
  return evaluate_actuation_sample(request).sample;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved
