#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace rate = multi_purpose_mpc_ros::mpcc_rate_resolved;

namespace
{

rate::LinearizationRequest nominal_request()
{
  rate::LinearizationRequest request;
  request.reference_lateral_m = 0.2;
  request.reference_lag_m = -0.1;
  request.reference_heading_rad = 0.08;
  request.reference_velocity_mps = 7.0;
  request.reference_progress_m = 12.0;
  request.reference_steering_rad = 0.15;
  request.reference_response_steering_rad = 0.10;
  request.reference_acceleration_mps2 = 0.4;
  request.reference_steering_rate_radps = -0.2;
  request.reference_virtual_progress_speed_mps = 6.8;
  request.reference_path_curvature_radpm = 0.04;
  request.wheelbase_m = 2.0;
  request.yaw_response_gain = 0.75;
  request.yaw_response_time_constant_sec = 0.13;
  request.stage_dt_sec = 0.12;
  return request;
}

}  // namespace

TEST(MpccRateResolved, ReferencePointSatisfiesAffineDynamics)
{
  const auto request = nominal_request();
  const auto linearization = rate::linearize_temporal_frenet(request);
  ASSERT_TRUE(linearization.has_value());

  Eigen::Matrix<double, rate::kStateDimension, 1> state;
  state <<
    request.reference_lateral_m, request.reference_lag_m,
    request.reference_heading_rad, request.reference_velocity_mps,
    request.reference_progress_m, request.reference_steering_rad,
    request.reference_response_steering_rad;
  Eigen::Matrix<double, rate::kInputDimension, 1> input;
  input <<
    request.reference_acceleration_mps2,
    request.reference_steering_rate_radps,
    request.reference_virtual_progress_speed_mps;
  const auto next =
    linearization->state_matrix * state +
    linearization->input_matrix * input -
    linearization->equality_offset;

  const double denominator =
    1.0 - request.reference_path_curvature_radpm *
    request.reference_lateral_m;
  EXPECT_NEAR(
    next[rate::kLateralIndex],
    request.reference_lateral_m + request.stage_dt_sec *
    request.reference_velocity_mps * std::sin(request.reference_heading_rad),
    1e-12);
  EXPECT_NEAR(
    next[rate::kLagIndex],
    request.reference_lag_m + request.stage_dt_sec *
    (request.reference_velocity_mps * std::cos(request.reference_heading_rad) /
    denominator - request.reference_virtual_progress_speed_mps),
    1e-12);
  EXPECT_NEAR(
    next[rate::kHeadingIndex],
    request.reference_heading_rad + request.yaw_response_gain *
    request.reference_velocity_mps / request.wheelbase_m *
    (std::tan(request.reference_response_steering_rad) *
    request.stage_dt_sec +
    (1.0 /
    (std::cos(request.reference_response_steering_rad) *
    std::cos(request.reference_response_steering_rad))) *
    ((request.reference_steering_rad -
    request.reference_response_steering_rad) *
    (request.stage_dt_sec - request.yaw_response_time_constant_sec *
    (1.0 - std::exp(
      -request.stage_dt_sec /
      request.yaw_response_time_constant_sec))) +
    request.reference_steering_rate_radps *
    (0.5 * request.stage_dt_sec * request.stage_dt_sec -
    request.yaw_response_time_constant_sec * request.stage_dt_sec +
    request.yaw_response_time_constant_sec *
    request.yaw_response_time_constant_sec *
    (1.0 - std::exp(
      -request.stage_dt_sec /
      request.yaw_response_time_constant_sec))))) -
    request.stage_dt_sec * request.reference_path_curvature_radpm *
    request.reference_virtual_progress_speed_mps,
    1e-12);
  EXPECT_NEAR(
    next[rate::kSteeringIndex],
    request.reference_steering_rad + request.stage_dt_sec *
    request.reference_steering_rate_radps,
    1e-12);
  EXPECT_NEAR(
    next[rate::kResponseSteeringIndex],
    request.reference_steering_rad +
    (request.reference_response_steering_rad -
    request.reference_steering_rad) * std::exp(
      -request.stage_dt_sec /
      request.yaw_response_time_constant_sec) +
    request.reference_steering_rate_radps *
    (request.stage_dt_sec - request.yaw_response_time_constant_sec *
    (1.0 - std::exp(
      -request.stage_dt_sec /
      request.yaw_response_time_constant_sec))),
    1e-12);
}

TEST(MpccRateResolved, SteeringRatePropagatesThroughResponseWithinStage)
{
  const auto linearization = rate::linearize_temporal_frenet(nominal_request());
  ASSERT_TRUE(linearization.has_value());
  EXPECT_GT(
    linearization->input_matrix(
      rate::kHeadingIndex, rate::kSteeringRateIndex),
    0.0);
  EXPECT_DOUBLE_EQ(
    linearization->input_matrix(
      rate::kSteeringIndex, rate::kSteeringRateIndex),
    nominal_request().stage_dt_sec);
  EXPECT_GT(
    linearization->state_matrix(
      rate::kHeadingIndex, rate::kResponseSteeringIndex),
    0.0);
  EXPECT_GT(
    linearization->state_matrix(
      rate::kHeadingIndex, rate::kSteeringIndex),
    0.0);
  EXPECT_GT(
    linearization->state_matrix(
      rate::kResponseSteeringIndex, rate::kSteeringIndex),
    0.0);
  EXPECT_GT(
    linearization->input_matrix(
      rate::kResponseSteeringIndex, rate::kSteeringRateIndex),
    0.0);
}

TEST(MpccRateResolved, RejectsInvalidGeometryAndTiming)
{
  auto request = nominal_request();
  request.wheelbase_m = 0.0;
  EXPECT_FALSE(rate::linearize_temporal_frenet(request).has_value());
  request = nominal_request();
  request.stage_dt_sec = request.maximum_stage_dt_sec + 0.01;
  EXPECT_FALSE(rate::linearize_temporal_frenet(request).has_value());
  request = nominal_request();
  request.reference_lateral_m =
    (1.0 - request.minimum_frenet_denominator) /
    request.reference_path_curvature_radpm + 0.01;
  EXPECT_FALSE(rate::linearize_temporal_frenet(request).has_value());
}

TEST(MpccRateResolved, SamplesIntermediateCertifiedSteering)
{
  const auto sample = rate::sample_actuation(rate::ActuationSampleRequest{
    0.10, 0.50, 0.025, 0.12, 0.6, 0.7, 2.0});
  ASSERT_TRUE(sample.has_value());
  EXPECT_NEAR(sample->steering_rad, 0.1125, 1e-12);
  EXPECT_NEAR(sample->curvature_radpm, std::tan(0.1125) / 2.0, 1e-12);
}

TEST(MpccRateResolved, RejectsUncertifiedActuationSamples)
{
  const auto rate_violation = rate::evaluate_actuation_sample(
    rate::ActuationSampleRequest{
      0.10, 0.80, 0.025, 0.12, 0.6, 0.7, 2.0});
  EXPECT_EQ(
    rate_violation.reason,
    rate::ActuationSampleReason::SteeringRateLimitViolation);
  EXPECT_FALSE(rate_violation.sample.has_value());

  const auto terminal_violation = rate::evaluate_actuation_sample(
    rate::ActuationSampleRequest{
      0.55, 0.50, 0.025, 0.12, 0.6, 0.7, 2.0});
  EXPECT_EQ(
    terminal_violation.reason,
    rate::ActuationSampleReason::TerminalSteeringLimitViolation);
  EXPECT_DOUBLE_EQ(terminal_violation.terminal_steering_rad, 0.61);

  const auto time_violation = rate::evaluate_actuation_sample(
    rate::ActuationSampleRequest{
      0.10, 0.50, 0.13, 0.12, 0.6, 0.7, 2.0});
  EXPECT_EQ(
    time_violation.reason,
    rate::ActuationSampleReason::PublicationAfterStageEnd);

  const auto initial_violation = rate::evaluate_actuation_sample(
    rate::ActuationSampleRequest{
      0.61, 0.0, 0.025, 0.12, 0.6, 0.7, 2.0});
  EXPECT_EQ(
    initial_violation.reason,
    rate::ActuationSampleReason::InitialSteeringLimitViolation);

  const auto nonfinite = rate::evaluate_actuation_sample(
    rate::ActuationSampleRequest{
      std::numeric_limits<double>::quiet_NaN(),
      0.0, 0.0, 0.12, 0.6, 0.7, 2.0});
  EXPECT_EQ(
    nonfinite.reason,
    rate::ActuationSampleReason::InitialSteeringNonfinite);

  EXPECT_FALSE(rate::sample_actuation(rate::ActuationSampleRequest{
    0.10, 0.80, 0.025, 0.12, 0.6, 0.7, 2.0}).has_value());
}

TEST(MpccRateResolved, SamplesCertifiedPiecewiseSequenceAtPublicationBoundary)
{
  const auto accepted = rate::evaluate_certified_actuation_sequence_sample(
    rate::CertifiedActuationSequenceSampleRequest{
      0.10, {0.40, -0.20}, {0.01, 0.02}, 0.025, 0.60, 2.0, 0.75});
  ASSERT_EQ(accepted.reason, rate::ActuationSampleReason::Accepted);
  ASSERT_TRUE(accepted.sample.has_value());
  EXPECT_NEAR(accepted.sample->steering_rad, 0.101, 1e-12);
  EXPECT_EQ(accepted.sampled_stage_index, 1U);
  EXPECT_NEAR(accepted.sampled_stage_elapsed_sec, 0.015, 1e-12);
  EXPECT_NEAR(accepted.certified_horizon_duration_sec, 0.03, 1e-12);

  const auto exact_boundary =
    rate::evaluate_certified_actuation_sequence_sample(
    rate::CertifiedActuationSequenceSampleRequest{
      0.10, {0.40, -0.20}, {0.01, 0.02}, 0.01, 0.60, 2.0, 0.75});
  ASSERT_EQ(exact_boundary.reason, rate::ActuationSampleReason::Accepted);
  ASSERT_TRUE(exact_boundary.sample.has_value());
  EXPECT_NEAR(exact_boundary.sample->steering_rad, 0.104, 1e-12);
  EXPECT_EQ(exact_boundary.sampled_stage_index, 0U);
  EXPECT_NEAR(exact_boundary.sampled_stage_elapsed_sec, 0.01, 1e-12);
}

TEST(MpccRateResolved, RejectsUncertifiedPiecewiseSequence)
{
  const auto uncertified =
    rate::evaluate_certified_actuation_sequence_sample(
    rate::CertifiedActuationSequenceSampleRequest{
      0.10, {0.10}, {0.12}, 0.025, 0.60, 2.0, 1.01});
  EXPECT_EQ(
    uncertified.reason,
    rate::ActuationSampleReason::SolverCertificateInvalid);

  const auto late = rate::evaluate_certified_actuation_sequence_sample(
    rate::CertifiedActuationSequenceSampleRequest{
      0.10, {0.10, -0.10}, {0.01, 0.02}, 0.031, 0.60, 2.0, 0.50});
  EXPECT_EQ(
    late.reason,
    rate::ActuationSampleReason::PublicationAfterHorizonEnd);

  const auto malformed = rate::evaluate_certified_actuation_sequence_sample(
    rate::CertifiedActuationSequenceSampleRequest{
      0.10, {0.10}, {0.01, 0.02}, 0.01, 0.60, 2.0, 0.50});
  EXPECT_EQ(
    malformed.reason,
    rate::ActuationSampleReason::StageSequenceInvalid);

  const auto sampled_violation =
    rate::evaluate_certified_actuation_sequence_sample(
    rate::CertifiedActuationSequenceSampleRequest{
      0.59, {2.0, -2.0}, {0.01, 0.02}, 0.025, 0.60, 2.0, 0.50});
  EXPECT_EQ(
    sampled_violation.reason,
    rate::ActuationSampleReason::SampledSteeringLimitViolation);
}
