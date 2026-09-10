#include "mpcc_vehicle_model_fixture.hpp"
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
  request.vehicle_model = multi_purpose_mpc_ros::test::vehicle_model();
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
    request.reference_response_steering_rad,
    request.reference_lateral_velocity_mps, request.reference_yaw_rate_radps;
  Eigen::Matrix<double, rate::kInputDimension, 1> input;
  input <<
    request.reference_acceleration_mps2,
    request.reference_steering_rate_radps,
    request.reference_virtual_progress_speed_mps;
  const auto next =
    linearization->state_matrix * state +
    linearization->input_matrix * input -
    linearization->equality_offset;
  const auto nonlinear = rate::evaluate_temporal_frenet_transition(request);
  ASSERT_TRUE(nonlinear.has_value());
  EXPECT_EQ(nonlinear->integration_substep_count, 24U);
  EXPECT_TRUE(next.isApprox(nonlinear->next_state, 1e-10));
}

TEST(MpccRateResolved, SteeringRatePropagatesThroughResponseWithinStage)
{
  const auto linearization = rate::linearize_temporal_frenet(nominal_request());
  ASSERT_TRUE(linearization.has_value());
  EXPECT_GT(
    linearization->input_matrix(
      rate::kHeadingIndex, rate::kSteeringRateIndex),
    0.0);
  EXPECT_NEAR(
    linearization->input_matrix(
      rate::kSteeringIndex, rate::kSteeringRateIndex),
    nominal_request().stage_dt_sec, 1e-8);
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

TEST(MpccRateResolved, AffineStateRowsUseExactAnalyticCoefficients)
{
  const auto request = nominal_request();
  const auto linearization = rate::linearize_temporal_frenet(request);
  ASSERT_TRUE(linearization.has_value());

  EXPECT_LT(
    linearization->input_matrix(rate::kVelocityIndex, rate::kAccelerationIndex),
    request.stage_dt_sec);
  EXPECT_NE(linearization->state_matrix(rate::kVelocityIndex, rate::kVelocityIndex), 1.0);
  EXPECT_DOUBLE_EQ(
    linearization->state_matrix(rate::kProgressIndex, rate::kProgressIndex),
    1.0);
  EXPECT_DOUBLE_EQ(
    linearization->input_matrix(
      rate::kProgressIndex, rate::kVirtualProgressSpeedIndex),
    request.stage_dt_sec);
  EXPECT_DOUBLE_EQ(
    linearization->state_matrix(rate::kSteeringIndex, rate::kSteeringIndex),
    1.0);
  EXPECT_DOUBLE_EQ(
    linearization->input_matrix(
      rate::kSteeringIndex, rate::kSteeringRateIndex),
    request.stage_dt_sec);

  // The nine-state tire/body rows are derivatives of the shared transition.
  // They are not the retired surrogate exponential or affine velocity rows.
  auto perturbed = request;
  constexpr double delta = 1e-5;
  perturbed.reference_lateral_velocity_mps += delta;
  const auto reference = rate::evaluate_temporal_frenet_transition(request);
  const auto next = rate::evaluate_temporal_frenet_transition(perturbed);
  ASSERT_TRUE(reference);
  ASSERT_TRUE(next);
  const auto tangent = reference->next_state +
    delta * linearization->state_matrix.col(rate::kLateralVelocityIndex);
  EXPECT_LT((next->next_state - tangent).norm(), 1e-9);
  for (int column = 0; column < rate::kStateDimension; ++column) {
    if (column != rate::kProgressIndex) {
      EXPECT_DOUBLE_EQ(
        linearization->state_matrix(rate::kProgressIndex, column), 0.0);
    }
  }
  for (int column = 0; column < rate::kInputDimension; ++column) {
    if (column != rate::kVirtualProgressSpeedIndex) {
      EXPECT_DOUBLE_EQ(
        linearization->input_matrix(rate::kProgressIndex, column), 0.0);
    }
  }
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

TEST(MpccRateResolved, VirtualProgressCannotChangeBodyMotion)
{
  for (double curvature : {-0.2, 0.0, 0.2}) {
    for (double speed : {0.0, 8.0}) {
      for (double progress_speed : {0.0, 4.0, 8.0, 12.0}) {
        auto request = nominal_request();
        request.reference_lateral_m = speed == 0.0 ? 0.0 : 0.2;
        request.reference_lag_m = -0.3;
        request.reference_heading_rad = 0.0;
        request.reference_progress_m = 0.0;
        request.reference_velocity_mps = speed;
        request.reference_steering_rad = 0.0;
        request.reference_response_steering_rad = 0.0;
        request.reference_steering_rate_radps = 0.0;
        request.reference_acceleration_mps2 = 0.0;
        request.reference_virtual_progress_speed_mps = progress_speed;
        request.reference_path_curvature_radpm = curvature;
        request.stage_dt_sec = 0.1;
        const auto result = rate::evaluate_temporal_frenet_transition(request);
        ASSERT_TRUE(result);
        const auto & next = result->next_state;
        const double angle = curvature * next[rate::kProgressIndex];
        const double frame_x = curvature == 0.0 ? next[rate::kProgressIndex] :
          std::sin(angle) / curvature;
        const double frame_y = curvature == 0.0 ? 0.0 :
          (1.0 - std::cos(angle)) / curvature;
        const double x = frame_x + std::cos(angle) * next[rate::kLagIndex] -
          std::sin(angle) * next[rate::kLateralIndex];
        const double y = frame_y + std::sin(angle) * next[rate::kLagIndex] +
          std::cos(angle) * next[rate::kLateralIndex];
        const auto physical = multi_purpose_mpc_ros::mpcc_vehicle_model::advance(
          {-0.3, request.reference_lateral_m, 0, speed, 0, 0, 0, 0},
          {0, 0}, request.vehicle_model, .1);
        ASSERT_TRUE(physical);
        EXPECT_NEAR(x, physical->state.x_m, 1e-10);
        EXPECT_NEAR(y, request.reference_lateral_m, 1e-10);
        EXPECT_NEAR(angle + next[rate::kHeadingIndex], 0.0, 1e-10);
      }
    }
  }
}

TEST(MpccRateResolved, PiecewiseCourseCannotDeflectStraightPhysicalMotion)
{
  namespace geometry = multi_purpose_mpc_ros::mpc_stage_geometry;
  // The first segment is from captured source6782. Its chord and interpolated
  // heading have different derivatives; later knots exercise a knot crossing.
  const auto knots = std::make_shared<const std::vector<geometry::CourseFrameKnot>>(
    std::vector<geometry::CourseFrameKnot>{
      {279.66607474985835, 89667.80949128, 43169.87420976, 0.26301156595296, 283},
      {280.6442844703521, 89668.75406172, 43170.12853424, 0.11716940361073, 284},
      {281.6442844703521, 89669.65, 43170.2, -0.02, 285},
      {283.6442844703521, 89671.6, 43170.1, -0.08, 286}});
  auto request = nominal_request();
  request.course_frame = {knots, knots->front().progress_m};
  request.reference_progress_m = 0.0;
  request.reference_lateral_m = -0.118312279259;
  request.reference_lag_m = -0.305091301859;
  request.reference_heading_rad = -0.235430920532;
  request.reference_velocity_mps = 7.4659609;
  request.reference_acceleration_mps2 = 1.3295945;
  request.reference_steering_rad = 0.0;
  request.reference_response_steering_rad = 0.0;
  request.reference_steering_rate_radps = 0.0;
  request.reference_path_curvature_radpm = -0.166609279;
  request.stage_dt_sec = 0.2;
  const auto start = geometry::sample_course_frame(*knots, knots->front().progress_m);
  ASSERT_TRUE(start);
  const double x0 = start->x_m + request.reference_lag_m * std::cos(start->heading_rad) -
    request.reference_lateral_m * std::sin(start->heading_rad);
  const double y0 = start->y_m + request.reference_lag_m * std::sin(start->heading_rad) +
    request.reference_lateral_m * std::cos(start->heading_rad);
  const double yaw = start->heading_rad + request.reference_heading_rad;
  const auto physical = multi_purpose_mpc_ros::mpcc_vehicle_model::advance(
    {x0, y0, yaw, request.reference_velocity_mps, 0, 0, 0, 0},
    {request.reference_acceleration_mps2, 0}, request.vehicle_model, request.stage_dt_sec);
  ASSERT_TRUE(physical);
  for (double progress_speed : {0.0, 3.0, 7.0, 10.83925988}) {
    request.reference_virtual_progress_speed_mps = progress_speed;
    const auto result = rate::evaluate_temporal_frenet_transition(request);
    ASSERT_TRUE(result);
    const auto & next = result->next_state;
    const auto end = geometry::sample_course_frame(
      *knots, request.course_frame.progress_origin_m + next[rate::kProgressIndex]);
    ASSERT_TRUE(end);
    const double x = end->x_m + next[rate::kLagIndex] * std::cos(end->heading_rad) -
      next[rate::kLateralIndex] * std::sin(end->heading_rad);
    const double y = end->y_m + next[rate::kLagIndex] * std::sin(end->heading_rad) +
      next[rate::kLateralIndex] * std::cos(end->heading_rad);
    EXPECT_NEAR(x, physical->state.x_m, 3e-10);
    EXPECT_NEAR(y, physical->state.y_m, 3e-10);
    EXPECT_NEAR(end->heading_rad + next[rate::kHeadingIndex], yaw, 1e-12);
    const auto tangent = rate::linearize_temporal_frenet(request);
    ASSERT_TRUE(tangent);
    EXPECT_TRUE(tangent->state_matrix.allFinite());
    EXPECT_TRUE(tangent->input_matrix.allFinite());
  }
  request.reference_virtual_progress_speed_mps = 40.0;
  EXPECT_FALSE(rate::evaluate_temporal_frenet_transition(request));
  request.course_frame.knots =
    std::make_shared<const std::vector<geometry::CourseFrameKnot>>();
  EXPECT_FALSE(rate::evaluate_temporal_frenet_transition(request));
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
