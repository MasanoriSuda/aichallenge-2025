#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"

#include <algorithm>
#include <cmath>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved
{

std::optional<Linearization> linearize_temporal_frenet(
  const LinearizationRequest & request) noexcept
{
  constexpr double half_pi = 1.57079632679489661923;
  if (
    !std::isfinite(request.reference_lateral_m) ||
    !std::isfinite(request.reference_lag_m) ||
    !std::isfinite(request.reference_heading_rad) ||
    !std::isfinite(request.reference_velocity_mps) ||
    request.reference_velocity_mps < 0.0 ||
    !std::isfinite(request.reference_progress_m) ||
    !std::isfinite(request.reference_steering_rad) ||
    std::abs(request.reference_steering_rad) >= half_pi ||
    !std::isfinite(request.reference_response_steering_rad) ||
    std::abs(request.reference_response_steering_rad) >= half_pi ||
    !std::isfinite(request.reference_acceleration_mps2) ||
    !std::isfinite(request.reference_steering_rate_radps) ||
    !std::isfinite(request.reference_virtual_progress_speed_mps) ||
    request.reference_virtual_progress_speed_mps < 0.0 ||
    !std::isfinite(request.reference_path_curvature_radpm) ||
    !std::isfinite(request.wheelbase_m) || request.wheelbase_m <= 0.0 ||
    !std::isfinite(request.yaw_response_gain) ||
    request.yaw_response_gain <= 0.0 ||
    !std::isfinite(request.yaw_response_time_constant_sec) ||
    request.yaw_response_time_constant_sec <= 0.0 ||
    !std::isfinite(request.stage_dt_sec) ||
    !std::isfinite(request.minimum_frenet_denominator) ||
    request.minimum_frenet_denominator <= 0.0 ||
    !std::isfinite(request.minimum_stage_dt_sec) ||
    request.minimum_stage_dt_sec <= 0.0 ||
    !std::isfinite(request.maximum_stage_dt_sec) ||
    request.maximum_stage_dt_sec < request.minimum_stage_dt_sec ||
    request.stage_dt_sec < request.minimum_stage_dt_sec ||
    request.stage_dt_sec > request.maximum_stage_dt_sec)
  {
    return std::nullopt;
  }

  const double lateral = request.reference_lateral_m;
  const double heading = request.reference_heading_rad;
  const double velocity = request.reference_velocity_mps;
  const double steering = request.reference_steering_rad;
  const double response_steering =
    request.reference_response_steering_rad;
  const double path_curvature = request.reference_path_curvature_radpm;
  const double progress_speed = request.reference_virtual_progress_speed_mps;
  const double denominator = 1.0 - path_curvature * lateral;
  if (
    !std::isfinite(denominator) ||
    denominator <= request.minimum_frenet_denominator)
  {
    return std::nullopt;
  }

  const double sin_heading = std::sin(heading);
  const double cos_heading = std::cos(heading);
  const double tan_response_steering = std::tan(response_steering);
  const double cos_response_steering = std::cos(response_steering);
  const double secant_squared_response_steering =
    1.0 / (cos_response_steering * cos_response_steering);
  const double yaw_velocity_jacobian =
    request.yaw_response_gain * tan_response_steering /
    request.wheelbase_m;
  const double yaw_response_jacobian =
    request.yaw_response_gain * velocity *
    secant_squared_response_steering / request.wheelbase_m;
  const double physical_progress_rate = velocity * cos_heading / denominator;

  using StateMatrix = Eigen::Matrix<double, kStateDimension, kStateDimension>;
  using InputMatrix = Eigen::Matrix<double, kStateDimension, kInputDimension>;
  using StateVector = Eigen::Matrix<double, kStateDimension, 1>;
  using InputVector = Eigen::Matrix<double, kInputDimension, 1>;

  StateMatrix continuous_state = StateMatrix::Zero();
  continuous_state(kLateralIndex, kHeadingIndex) = velocity * cos_heading;
  continuous_state(kLateralIndex, kVelocityIndex) = sin_heading;
  continuous_state(kLagIndex, kLateralIndex) =
    velocity * cos_heading * path_curvature /
    (denominator * denominator);
  continuous_state(kLagIndex, kHeadingIndex) =
    -velocity * sin_heading / denominator;
  continuous_state(kLagIndex, kVelocityIndex) = cos_heading / denominator;
  continuous_state(kHeadingIndex, kVelocityIndex) = yaw_velocity_jacobian;
  continuous_state(kHeadingIndex, kResponseSteeringIndex) =
    yaw_response_jacobian;
  continuous_state(kResponseSteeringIndex, kSteeringIndex) =
    1.0 / request.yaw_response_time_constant_sec;
  continuous_state(kResponseSteeringIndex, kResponseSteeringIndex) =
    -1.0 / request.yaw_response_time_constant_sec;

  InputMatrix continuous_input = InputMatrix::Zero();
  continuous_input(kLagIndex, kVirtualProgressSpeedIndex) = -1.0;
  continuous_input(kHeadingIndex, kVirtualProgressSpeedIndex) = -path_curvature;
  continuous_input(kVelocityIndex, kAccelerationIndex) = 1.0;
  continuous_input(kProgressIndex, kVirtualProgressSpeedIndex) = 1.0;
  continuous_input(kSteeringIndex, kSteeringRateIndex) = 1.0;

  const double dt = request.stage_dt_sec;
  Linearization result;
  result.state_matrix = StateMatrix::Identity() + dt * continuous_state;
  result.input_matrix = dt * continuous_input;
  const double yaw_response_decay = std::exp(
    -dt / request.yaw_response_time_constant_sec);
  const double response_integral_from_response_sec =
    request.yaw_response_time_constant_sec * (1.0 - yaw_response_decay);
  const double response_integral_from_steering_sec =
    dt - response_integral_from_response_sec;
  const double response_integral_from_rate_sec2 =
    0.5 * dt * dt - request.yaw_response_time_constant_sec * dt +
    request.yaw_response_time_constant_sec *
    request.yaw_response_time_constant_sec * (1.0 - yaw_response_decay);

  // Steering rate is held constant over a stage.  The response state sees
  // the resulting steering ramp during that same stage, and yaw integrates
  // that response.  Forward Euler previously omitted both paths, introducing
  // an artificial full-stage delay.  The optimizer then certified a vehicle
  // which turned later than AWSIM and could accept a horizon whose real
  // footprint reached the wall first.
  result.state_matrix(kHeadingIndex, kResponseSteeringIndex) =
    yaw_response_jacobian * response_integral_from_response_sec;
  result.state_matrix(kHeadingIndex, kSteeringIndex) =
    yaw_response_jacobian * response_integral_from_steering_sec;
  result.input_matrix(kHeadingIndex, kSteeringRateIndex) =
    yaw_response_jacobian * response_integral_from_rate_sec2;
  result.state_matrix.row(kResponseSteeringIndex).setZero();
  result.state_matrix(kResponseSteeringIndex, kSteeringIndex) =
    1.0 - yaw_response_decay;
  result.state_matrix(
    kResponseSteeringIndex, kResponseSteeringIndex) = yaw_response_decay;
  result.input_matrix(kResponseSteeringIndex, kSteeringRateIndex) =
    response_integral_from_steering_sec;
  result.stage_dt_sec = dt;

  const StateVector reference_state = (StateVector() <<
    lateral, request.reference_lag_m, heading, velocity,
    request.reference_progress_m, steering, response_steering).finished();
  const InputVector reference_input = (InputVector() <<
    request.reference_acceleration_mps2,
    request.reference_steering_rate_radps, progress_speed).finished();
  const double integrated_response_delta_rad_sec =
    (steering - response_steering) *
    response_integral_from_steering_sec +
    request.reference_steering_rate_radps *
    response_integral_from_rate_sec2;
  const double integrated_yaw_response_rad_sec =
    tan_response_steering * dt +
    secant_squared_response_steering * integrated_response_delta_rad_sec;
  const StateVector next_reference = (StateVector() <<
    lateral + dt * velocity * sin_heading,
    request.reference_lag_m + dt * (physical_progress_rate - progress_speed),
    heading + request.yaw_response_gain * velocity *
    integrated_yaw_response_rad_sec / request.wheelbase_m -
    dt * path_curvature * progress_speed,
    velocity + dt * request.reference_acceleration_mps2,
    request.reference_progress_m + dt * progress_speed,
    steering + dt * request.reference_steering_rate_radps,
    steering + (response_steering - steering) *
    yaw_response_decay + request.reference_steering_rate_radps *
    response_integral_from_steering_sec).finished();
  result.equality_offset =
    result.state_matrix * reference_state +
    result.input_matrix * reference_input - next_reference;

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
