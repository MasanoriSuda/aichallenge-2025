#include <gtest/gtest.h>

#include <multi_purpose_mpc_ros/mpcc_progress.hpp>

#include <cmath>
#include <vector>

namespace
{

using multi_purpose_mpc_ros::mpcc_progress::Config;
using multi_purpose_mpc_ros::mpcc_progress::LinearizationRequest;

TEST(MpccProgress, ActivatesForDynamicEscapeOutsideOvertakeLinePhase)
{
  using multi_purpose_mpc_ros::mpcc_progress::ActivationRequest;
  using multi_purpose_mpc_ros::mpcc_progress::ActivationSource;
  using multi_purpose_mpc_ros::mpcc_progress::resolve_activation;
  const auto resolution = resolve_activation(
    ActivationRequest{true, true, false, true});
  EXPECT_TRUE(resolution.requested);
  EXPECT_EQ(resolution.source, ActivationSource::DynamicObstacleEscape);
}

TEST(MpccProgress, KeepsOrdinaryCruiseOnLegacyMpcInOvertakeOnlyScope)
{
  using multi_purpose_mpc_ros::mpcc_progress::ActivationRequest;
  using multi_purpose_mpc_ros::mpcc_progress::ActivationSource;
  using multi_purpose_mpc_ros::mpcc_progress::resolve_activation;
  const auto resolution = resolve_activation(
    ActivationRequest{true, true, false, false});
  EXPECT_FALSE(resolution.requested);
  EXPECT_EQ(resolution.source, ActivationSource::OvertakeScopeInactive);
}

TEST(MpccProgress, GlobalScopeStillActivatesOrdinaryCruise)
{
  using multi_purpose_mpc_ros::mpcc_progress::ActivationRequest;
  using multi_purpose_mpc_ros::mpcc_progress::ActivationSource;
  using multi_purpose_mpc_ros::mpcc_progress::resolve_activation;
  const auto resolution = resolve_activation(
    ActivationRequest{true, false, false, false});
  EXPECT_TRUE(resolution.requested);
  EXPECT_EQ(resolution.source, ActivationSource::Global);
}

TEST(MpccProgress, StraightLinearizationAdvancesPhysicalProgress)
{
  const auto result = multi_purpose_mpc_ros::mpcc_progress::linearize_temporal_frenet(
    LinearizationRequest{0.0, 0.0, 10.0, 5.0, 0.0, 0.0, 0.5, Config{}});
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->stage_dt_sec, 0.1, 1e-12);
  EXPECT_NEAR(result->state_matrix(0, 1), 0.5, 1e-12);
  EXPECT_NEAR(result->input_matrix(2, 0), 0.1, 1e-12);

  const Eigen::Vector3d state(0.0, 0.0, 10.0);
  const Eigen::Vector2d input(5.0, 0.0);
  const Eigen::Vector3d next =
    result->state_matrix * state + result->input_matrix * input -
    result->equality_offset;
  EXPECT_NEAR(next[0], 0.0, 1e-12);
  EXPECT_NEAR(next[1], 0.0, 1e-12);
  EXPECT_NEAR(next[2], 10.5, 1e-12);
}

TEST(MpccProgress, CurvedReferenceIsAnEquilibriumInContourAndHeading)
{
  const auto result = multi_purpose_mpc_ros::mpcc_progress::linearize_temporal_frenet(
    LinearizationRequest{0.0, 0.0, 20.0, 5.0, 0.1, 0.1, 0.5, Config{}});
  ASSERT_TRUE(result.has_value());
  const Eigen::Vector3d state(0.0, 0.0, 20.0);
  const Eigen::Vector2d input(5.0, 0.1);
  const Eigen::Vector3d next =
    result->state_matrix * state + result->input_matrix * input -
    result->equality_offset;
  EXPECT_NEAR(next[0], 0.0, 1e-12);
  EXPECT_NEAR(next[1], 0.0, 1e-12);
  EXPECT_NEAR(next[2], 20.5, 1e-12);
}

TEST(MpccProgress, RejectsSingularFrenetReference)
{
  const auto result = multi_purpose_mpc_ros::mpcc_progress::linearize_temporal_frenet(
    LinearizationRequest{10.0, 0.0, 0.0, 5.0, 0.1, 0.1, 0.5, Config{}});
  EXPECT_FALSE(result.has_value());
}

TEST(MpccProgress, SeparatesPathAndInputCurvatureDuringRelinearization)
{
  const auto result = multi_purpose_mpc_ros::mpcc_progress::linearize_temporal_frenet(
    LinearizationRequest{0.0, 0.0, 20.0, 5.0, 0.1, 0.15, 0.5, Config{}});
  ASSERT_TRUE(result.has_value());
  const Eigen::Vector3d state(0.0, 0.0, 20.0);
  const Eigen::Vector2d input(5.0, 0.15);
  const Eigen::Vector3d next =
    result->state_matrix * state + result->input_matrix * input -
    result->equality_offset;
  EXPECT_NEAR(next[0], 0.0, 1e-12);
  EXPECT_NEAR(next[1], 0.025, 1e-12);
  EXPECT_NEAR(next[2], 20.5, 1e-12);
}

TEST(MpccProgress, ExtendedModelAdvancesVelocityAndVirtualProgress)
{
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedLinearizationRequest;
  using multi_purpose_mpc_ros::mpcc_progress::kExtendedInputDimension;
  using multi_purpose_mpc_ros::mpcc_progress::kExtendedStateDimension;
  const auto result =
    multi_purpose_mpc_ros::mpcc_progress::linearize_extended_temporal_frenet(
    ExtendedLinearizationRequest{
      0.0, 0.0, 0.0, 5.0, 10.0, 1.0, 0.0, 0.0, 5.0, 0.5, Config{}});
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->stage_dt_sec, 0.1, 1e-12);
  Eigen::Matrix<double, kExtendedStateDimension, 1> state;
  state << 0.0, 0.0, 0.0, 5.0, 10.0;
  Eigen::Matrix<double, kExtendedInputDimension, 1> input;
  input << 1.0, 0.0, 5.0;
  const auto next =
    result->state_matrix * state + result->input_matrix * input -
    result->equality_offset;
  EXPECT_NEAR(next[0], 0.0, 1e-12);
  EXPECT_NEAR(next[1], 0.0, 1e-12);
  EXPECT_NEAR(next[2], 0.0, 1e-12);
  EXPECT_NEAR(next[3], 5.1, 1e-12);
  EXPECT_NEAR(next[4], 10.5, 1e-12);
}

TEST(MpccProgress, ExtendedModelExposesPhysicalVirtualProgressLag)
{
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedLinearizationRequest;
  using multi_purpose_mpc_ros::mpcc_progress::kExtendedInputDimension;
  using multi_purpose_mpc_ros::mpcc_progress::kExtendedStateDimension;
  const auto result =
    multi_purpose_mpc_ros::mpcc_progress::linearize_extended_temporal_frenet(
    ExtendedLinearizationRequest{
      0.0, 0.0, 0.0, 5.0, 10.0, 0.0, 0.0, 0.0, 4.0, 0.5, Config{}});
  ASSERT_TRUE(result.has_value());
  Eigen::Matrix<double, kExtendedStateDimension, 1> state;
  state << 0.0, 0.0, 0.0, 5.0, 10.0;
  Eigen::Matrix<double, kExtendedInputDimension, 1> input;
  input << 0.0, 0.0, 4.0;
  const auto next =
    result->state_matrix * state + result->input_matrix * input -
    result->equality_offset;
  EXPECT_NEAR(next[1], 0.1, 1e-12);
  EXPECT_NEAR(next[4], 10.4, 1e-12);
}

TEST(MpccProgress, WallAwareReferenceStaysInsideWideCorridor)
{
  using multi_purpose_mpc_ros::mpcc_progress::WallAwareTrackingReferenceRequest;
  const auto result =
    multi_purpose_mpc_ros::mpcc_progress::resolve_wall_aware_tracking_reference(
      WallAwareTrackingReferenceRequest{0.98, -1.0, 1.0, 0.15, 0.25});
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->reference_adjusted);
  EXPECT_NEAR(result->reference_lateral_m, 0.85, 1e-12);
  EXPECT_NEAR(result->achieved_reserve_m, 0.15, 1e-12);
  EXPECT_DOUBLE_EQ(result->weight_scale, 1.0);
}

TEST(MpccProgress, WallAwareReferenceSoftensWhenCorridorIsNarrow)
{
  using multi_purpose_mpc_ros::mpcc_progress::WallAwareTrackingReferenceRequest;
  const auto result =
    multi_purpose_mpc_ros::mpcc_progress::resolve_wall_aware_tracking_reference(
      WallAwareTrackingReferenceRequest{0.08, -0.10, 0.10, 0.15, 0.25});
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->reference_adjusted);
  EXPECT_NEAR(result->reference_lateral_m, 0.0, 1e-12);
  EXPECT_NEAR(result->achieved_reserve_m, 0.10, 1e-12);
  EXPECT_NEAR(result->weight_scale, 0.75, 1e-12);
}

TEST(MpccProgress, WallAwareReferenceDoesNotAddASecondHardConstraint)
{
  using multi_purpose_mpc_ros::mpcc_progress::WallAwareTrackingReferenceRequest;
  const auto result =
    multi_purpose_mpc_ros::mpcc_progress::resolve_wall_aware_tracking_reference(
      WallAwareTrackingReferenceRequest{0.4, 0.4, 0.4, 0.15, 0.25});
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->reference_lateral_m, 0.4);
  EXPECT_DOUBLE_EQ(result->achieved_reserve_m, 0.0);
  EXPECT_DOUBLE_EQ(result->weight_scale, 0.25);
}

TEST(MpccProgress, WallAwareReferenceRejectsMalformedBounds)
{
  using multi_purpose_mpc_ros::mpcc_progress::WallAwareTrackingReferenceRequest;
  EXPECT_FALSE(
    multi_purpose_mpc_ros::mpcc_progress::resolve_wall_aware_tracking_reference(
      WallAwareTrackingReferenceRequest{0.0, 1.0, -1.0, 0.15, 0.25}).has_value());
}

TEST(MpccProgress, CommittedPassRaisesVelocityCostWithoutRelaxingCap)
{
  using multi_purpose_mpc_ros::mpcc_progress::VelocityHorizonRequest;
  Config config;
  config.stage_velocity_weight = 8.0;
  config.committed_stage_velocity_weight = 24.0;
  config.terminal_velocity_weight = 12.0;
  config.committed_terminal_velocity_weight = 45.0;
  const auto result = multi_purpose_mpc_ros::mpcc_progress::resolve_velocity_horizon(
    VelocityHorizonRequest{{5.0, 6.0}, {4.5, 5.5}, true, config});
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->reference_velocity_mps[0], 4.5);
  EXPECT_DOUBLE_EQ(result->hard_cap_velocity_mps[0], 4.5);
  EXPECT_DOUBLE_EQ(result->stage_weight[0], 24.0);
  EXPECT_DOUBLE_EQ(result->terminal_target_velocity_mps, 5.5);
  EXPECT_DOUBLE_EQ(result->terminal_weight, 45.0);
}

TEST(MpccProgress, ConvertsExtendedSolutionToEstablishedLayout)
{
  // N=2: 5 states x 3 stages followed by 3 inputs x 2 stages.
  Eigen::VectorXd extended(21);
  extended <<
    0.0, 0.0, 0.01, 4.0, 10.0,
    0.2, 0.0, 0.02, 4.5, 10.5,
    0.4, 0.0, 0.03, 5.0, 11.0,
    1.0, 0.10, 4.5,
    0.5, 0.20, 5.0;
  const auto legacy =
    multi_purpose_mpc_ros::mpcc_progress::convert_extended_solution_to_legacy(
    extended, 2, 0.0);
  ASSERT_TRUE(legacy.has_value());
  ASSERT_EQ(legacy->size(), 13);
  EXPECT_DOUBLE_EQ((*legacy)[3], 0.2);
  EXPECT_DOUBLE_EQ((*legacy)[4], 0.02);
  EXPECT_DOUBLE_EQ((*legacy)[5], 10.5);
  EXPECT_DOUBLE_EQ((*legacy)[9], 4.5);
  EXPECT_DOUBLE_EQ((*legacy)[10], 0.10);
  EXPECT_DOUBLE_EQ((*legacy)[11], 5.0);
  EXPECT_DOUBLE_EQ((*legacy)[12], 0.20);
}

TEST(MpccProgress, ExtractsTypedActuationWithoutLosingOptimizedAcceleration)
{
  Eigen::VectorXd extended(21);
  extended <<
    0.0, 0.0, 0.01, 4.0, 10.0,
    0.2, 0.0, 0.02, 4.5, 10.5,
    0.4, 0.0, 0.03, 5.0, 11.0,
    1.0, 0.10, 4.5,
    0.5, 0.20, 5.0;

  const auto proposal =
    multi_purpose_mpc_ros::mpcc_progress::extract_actuation_proposal(
    extended, 2, 1.1);

  ASSERT_TRUE(proposal.has_value());
  EXPECT_DOUBLE_EQ(proposal->predicted_speed_mps, 4.5);
  EXPECT_DOUBLE_EQ(proposal->acceleration_mps2, 1.0);
  EXPECT_DOUBLE_EQ(proposal->curvature_radpm, 0.10);
  EXPECT_NEAR(proposal->steering_tire_angle_rad, std::atan(0.11), 1e-12);
  EXPECT_DOUBLE_EQ(proposal->virtual_progress_speed_mps, 4.5);
}

TEST(MpccProgress, RejectsMalformedOrNonfiniteActuationProposal)
{
  Eigen::VectorXd malformed = Eigen::VectorXd::Zero(20);
  EXPECT_FALSE(
    multi_purpose_mpc_ros::mpcc_progress::extract_actuation_proposal(
      malformed, 2, 1.1).has_value());

  Eigen::VectorXd nonfinite = Eigen::VectorXd::Zero(21);
  nonfinite[15] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
    multi_purpose_mpc_ros::mpcc_progress::extract_actuation_proposal(
      nonfinite, 2, 1.1).has_value());
  EXPECT_FALSE(
    multi_purpose_mpc_ros::mpcc_progress::extract_actuation_proposal(
    Eigen::VectorXd::Zero(21), 2, 0.0).has_value());
}

TEST(MpccProgress, ExtractsFiveStatePoseWithoutReconstructingHeading)
{
  Eigen::VectorXd extended(21);
  extended <<
    0.0, 0.0, 0.01, 4.0, 0.0,
    0.2, 0.0, -0.17, 4.5, 0.5,
    0.4, 0.0, 0.23, 5.0, 1.0,
    1.0, 0.10, 4.5,
    0.5, 0.20, 5.0;

  const auto trajectory =
    multi_purpose_mpc_ros::mpcc_progress::extract_extended_execution_trajectory(
    extended, 2, {0.5, 1.0}, {-0.5, -0.5}, {0.8, 0.8}, 348.0, 1e-5);

  ASSERT_TRUE(trajectory.has_value());
  ASSERT_EQ(trajectory->heading_offset_rad.size(), 2U);
  EXPECT_DOUBLE_EQ(trajectory->lateral_m[0], 0.2);
  EXPECT_DOUBLE_EQ(trajectory->heading_offset_rad[0], -0.17);
  EXPECT_DOUBLE_EQ(trajectory->heading_offset_rad[1], 0.23);
  EXPECT_DOUBLE_EQ(trajectory->velocity_mps[1], 5.0);
  EXPECT_DOUBLE_EQ(trajectory->progress_m[1], 349.0);
  EXPECT_NEAR(trajectory->minimum_lateral_bound_reserve_m, 0.4, 1e-12);
}

TEST(MpccProgress, RejectsNonfiniteFiveStatePoseWithStageProvenance)
{
  Eigen::VectorXd extended = Eigen::VectorXd::Zero(21);
  extended[5 + multi_purpose_mpc_ros::mpcc_progress::kExtendedHeadingIndex] =
    std::numeric_limits<double>::quiet_NaN();
  multi_purpose_mpc_ros::mpcc_progress::ExecutionTrajectoryDiagnostic diagnostic;

  EXPECT_FALSE(
    multi_purpose_mpc_ros::mpcc_progress::extract_extended_execution_trajectory(
    extended, 2, {0.5, 1.0}, {-0.5, -0.5}, {0.8, 0.8}, 348.0, 1e-5,
    &diagnostic).has_value());
  EXPECT_EQ(
    diagnostic.rejection,
    multi_purpose_mpc_ros::mpcc_progress::ExecutionTrajectoryRejection::NonFiniteState);
  EXPECT_EQ(diagnostic.stage, 0);
}

TEST(MpccProgress, PreservesFiniteSignedBoundaryValuesForLaterCertification)
{
  Eigen::VectorXd extended = Eigen::VectorXd::Zero(21);
  extended[5 + multi_purpose_mpc_ros::mpcc_progress::kExtendedVelocityIndex] = -1e-7;
  extended[15 +
    multi_purpose_mpc_ros::mpcc_progress::kExtendedVirtualProgressSpeedIndex] = -2e-7;

  const auto proposal =
    multi_purpose_mpc_ros::mpcc_progress::extract_actuation_proposal(
    extended, 2, 1.1);

  ASSERT_TRUE(proposal.has_value());
  EXPECT_DOUBLE_EQ(proposal->predicted_speed_mps, -1e-7);
  EXPECT_DOUBLE_EQ(proposal->virtual_progress_speed_mps, -2e-7);
}

TEST(MpccProgress, RestoresAbsoluteProgressFromLocalExtendedSolution)
{
  Eigen::VectorXd extended(21);
  extended <<
    0.0, 0.0, 0.01, 4.0, 0.0,
    0.2, 0.0, 0.02, 4.5, 0.5,
    0.4, 0.0, 0.03, 5.0, 1.0,
    1.0, 0.10, 4.5,
    0.5, 0.20, 5.0;
  const auto legacy =
    multi_purpose_mpc_ros::mpcc_progress::convert_extended_solution_to_legacy(
    extended, 2, 348.0);
  ASSERT_TRUE(legacy.has_value());
  EXPECT_DOUBLE_EQ((*legacy)[2], 348.0);
  EXPECT_DOUBLE_EQ((*legacy)[5], 348.5);
  EXPECT_DOUBLE_EQ((*legacy)[8], 349.0);
}

TEST(MpccProgress, RebasesExtendedWarmStartToCurrentProgressOrigin)
{
  Eigen::VectorXd extended = Eigen::VectorXd::Zero(21);
  extended[4] = 0.5;
  extended[9] = 1.0;
  extended[14] = 1.5;
  ASSERT_TRUE(
    multi_purpose_mpc_ros::mpcc_progress::rebase_extended_progress_warm_start(
      extended, 2, 348.0, 348.4));
  EXPECT_NEAR(extended[4], 0.1, 1e-12);
  EXPECT_NEAR(extended[9], 0.6, 1e-12);
  EXPECT_NEAR(extended[14], 1.1, 1e-12);
  EXPECT_DOUBLE_EQ(extended[15], 0.0);
}

TEST(MpccProgress, ExtendedSolverCircuitBreakerUsesBoundedCooldown)
{
  multi_purpose_mpc_ros::mpcc_progress::ExtendedSolverCircuitBreaker breaker;
  EXPECT_FALSE(breaker.active(10.0));
  breaker.record_failure(10.0, 0.75);
  EXPECT_TRUE(breaker.active(10.74));
  EXPECT_FALSE(breaker.active(10.75));
  breaker.record_failure(11.0, 0.75);
  breaker.record_success();
  EXPECT_FALSE(breaker.active(11.1));
}

TEST(MpccProgress, ExtendedSolverReentryRequiresConsecutiveProbeSuccessesAfterFailure)
{
  multi_purpose_mpc_ros::mpcc_progress::ExtendedSolverReentryGate gate;

  auto result = gate.record_success(3U);
  EXPECT_TRUE(result.accept_solution);
  EXPECT_FALSE(result.requalifying);

  gate.record_failure();
  EXPECT_TRUE(gate.requalification_required());
  result = gate.record_success(3U);
  EXPECT_FALSE(result.accept_solution);
  EXPECT_TRUE(result.requalifying);
  EXPECT_EQ(result.consecutive_successes, 1U);
  EXPECT_EQ(gate.consecutive_successes(), 1U);

  result = gate.record_success(3U);
  EXPECT_FALSE(result.accept_solution);
  EXPECT_EQ(result.consecutive_successes, 2U);

  result = gate.record_success(3U);
  EXPECT_TRUE(result.accept_solution);
  EXPECT_FALSE(result.requalifying);
  EXPECT_EQ(result.consecutive_successes, 3U);
  EXPECT_FALSE(gate.requalification_required());
  EXPECT_EQ(gate.consecutive_successes(), 0U);
}

TEST(MpccProgress, ExtendedSolverReentryFailureRestartsProbeStreak)
{
  multi_purpose_mpc_ros::mpcc_progress::ExtendedSolverReentryGate gate;
  gate.record_failure();
  EXPECT_FALSE(gate.record_success(3U).accept_solution);
  EXPECT_EQ(gate.consecutive_successes(), 1U);

  gate.record_failure();
  EXPECT_EQ(gate.consecutive_successes(), 0U);
  auto result = gate.record_success(1U);
  EXPECT_TRUE(result.accept_solution);
  EXPECT_FALSE(gate.requalification_required());
}

TEST(MpccProgress, ExtendedModeHandoffSmoothsOnlyWithinCurrentHardBounds)
{
  multi_purpose_mpc_ros::mpcc_progress::ExtendedModeHandoff handoff;
  auto result = handoff.resolve_velocity(true, 10.0, 6.0, 0.0, 8.0, 0.20);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->active);
  EXPECT_DOUBLE_EQ(result->velocity_mps, 6.0);

  result = handoff.resolve_velocity(false, 10.0, 4.0, 0.0, 8.0, 0.20);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->active);
  EXPECT_DOUBLE_EQ(result->velocity_mps, 6.0);
  result = handoff.resolve_velocity(false, 10.1, 4.0, 0.0, 8.0, 0.20);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->active);
  EXPECT_NEAR(result->velocity_mps, 5.0, 1e-12);
  result = handoff.resolve_velocity(false, 10.2, 4.0, 0.0, 8.0, 0.20);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->active);
  EXPECT_DOUBLE_EQ(result->velocity_mps, 4.0);

  // A lower safety cap is not blended through.
  result = handoff.resolve_velocity(true, 10.3, 7.0, 0.0, 3.0, 0.20);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->velocity_mps, 3.0);
}

TEST(MpccProgress, ExtendedModeHandoffRejectsMalformedBounds)
{
  multi_purpose_mpc_ros::mpcc_progress::ExtendedModeHandoff handoff;
  EXPECT_FALSE(handoff.resolve_velocity(
    true, 10.0, 5.0, 6.0, 4.0, 0.15).has_value());
}

TEST(MpccProgress, BuildsUnwrappedProgressReference)
{
  const auto result = multi_purpose_mpc_ros::mpcc_progress::build_progress_reference(
    99.5, std::vector<double>{0.4, 0.6, 0.5});
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 4U);
  EXPECT_NEAR(result->at(0), 99.5, 1e-12);
  EXPECT_NEAR(result->at(3), 101.0, 1e-12);
}

TEST(MpccProgress, NormalizesCircularSeamWithoutExpandingItToNominalSpacing)
{
  Config config;
  config.minimum_reference_speed_mps = 0.5;
  config.minimum_stage_dt_sec = 0.01;
  const auto result = multi_purpose_mpc_ros::mpcc_progress::resolve_stage_distances(
    std::vector<double>{0.999, 0.0, 1.001}, config);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->normalized_stage_count, 1U);
  EXPECT_NEAR(result->minimum_stage_distance_m, 0.005, 1e-12);
  ASSERT_EQ(result->distance_m.size(), 3U);
  EXPECT_NEAR(result->distance_m[0], 0.999, 1e-12);
  EXPECT_NEAR(result->distance_m[1], 0.005, 1e-12);
  EXPECT_NEAR(result->distance_m[2], 1.001, 1e-12);

  const auto progress = multi_purpose_mpc_ros::mpcc_progress::build_progress_reference(
    348.0, result->distance_m);
  ASSERT_TRUE(progress.has_value());
  EXPECT_NEAR(progress->back(), 350.005, 1e-12);
}

TEST(MpccProgress, RejectsCorruptStageDistanceInsteadOfRepairingIt)
{
  const auto result = multi_purpose_mpc_ros::mpcc_progress::resolve_stage_distances(
    std::vector<double>{1.0, -0.1, 1.0}, Config{});
  EXPECT_FALSE(result.has_value());
}

TEST(MpccProgress, DetectsLapProgressWrapForWarmStartReset)
{
  EXPECT_TRUE(multi_purpose_mpc_ros::mpcc_progress::progress_origin_discontinuous(
    348.0, 0.0, 12.0));
  EXPECT_FALSE(multi_purpose_mpc_ros::mpcc_progress::progress_origin_discontinuous(
    348.0, 349.0, 12.0));
}

TEST(MpccProgress, TrustRegionKeepsMeasuredProgressFeasible)
{
  Config config;
  config.trust_region_backward_m = 3.0;
  config.trust_region_forward_m = 1.0;
  const auto result = multi_purpose_mpc_ros::mpcc_progress::resolve_progress_bounds(
    10.0, 20.0, config);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->lower_m, 10.0);
  EXPECT_DOUBLE_EQ(result->upper_m, 21.0);
}

TEST(MpccProgress, ProgressRewardMovesCostAheadOfLagReference)
{
  Config config;
  config.lag_weight = 100.0;
  config.progress_reward_weight = 20.0;
  const auto result = multi_purpose_mpc_ros::mpcc_progress::resolve_progress_cost(
    5.0, false, config);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->quadratic_weight, 100.0);
  EXPECT_DOUBLE_EQ(result->linear_coefficient, -520.0);
  const double unconstrained_optimum =
    -result->linear_coefficient / result->quadratic_weight;
  EXPECT_NEAR(unconstrained_optimum, 5.2, 1e-12);
}

TEST(MpccProgress, DampsRtiSqpLinearizationPoint)
{
  Eigen::VectorXd previous(3);
  previous << 0.0, 2.0, 4.0;
  Eigen::VectorXd solution(3);
  solution << 2.0, 4.0, 8.0;
  const auto result = multi_purpose_mpc_ros::mpcc_progress::damp_rti_sqp_iterate(
    previous, solution, 0.65);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR((*result)[0], 1.3, 1e-12);
  EXPECT_NEAR((*result)[1], 3.3, 1e-12);
  EXPECT_NEAR((*result)[2], 6.6, 1e-12);
}

TEST(MpccProgress, RejectsMalformedRtiSqpUpdate)
{
  const Eigen::VectorXd finite = Eigen::VectorXd::Ones(3);
  EXPECT_FALSE(multi_purpose_mpc_ros::mpcc_progress::damp_rti_sqp_iterate(
    finite, Eigen::VectorXd::Ones(2), 0.65).has_value());
  EXPECT_FALSE(multi_purpose_mpc_ros::mpcc_progress::damp_rti_sqp_iterate(
    finite, finite, 0.0).has_value());
}

TEST(MpccProgress, RefinesOnlyWhenNonlinearOrBoundRiskNeedsIt)
{
  using multi_purpose_mpc_ros::mpcc_progress::RtiRefinementDecision;
  using multi_purpose_mpc_ros::mpcc_progress::RtiRefinementRequest;
  using multi_purpose_mpc_ros::mpcc_progress::resolve_rti_refinement;
  RtiRefinementRequest request;
  request.progress_mode_active = true;
  request.configured_iterations = 2;
  request.minimum_lateral_bound_reserve_m = 0.30;
  request.minimum_bound_reserve_threshold_m = 0.12;
  request.lateral_defect_m = 0.02;
  request.lateral_defect_threshold_m = 0.08;
  request.heading_defect_rad = 0.01;
  request.heading_defect_threshold_rad = 0.04;
  request.maximum_curvature_radpm = 0.02;
  request.curvature_threshold_radpm = 0.08;
  request.elapsed_ms = 2.0;
  request.refinement_start_deadline_ms = 12.0;

  EXPECT_EQ(resolve_rti_refinement(request), RtiRefinementDecision::SkipCondition);
  request.minimum_lateral_bound_reserve_m = 0.10;
  EXPECT_EQ(resolve_rti_refinement(request), RtiRefinementDecision::Refine);
  request.minimum_lateral_bound_reserve_m = 0.30;
  request.maximum_curvature_radpm = 0.10;
  EXPECT_EQ(resolve_rti_refinement(request), RtiRefinementDecision::Refine);
}

TEST(MpccProgress, DoesNotStartRefinementAfterDeadline)
{
  using multi_purpose_mpc_ros::mpcc_progress::RtiRefinementDecision;
  multi_purpose_mpc_ros::mpcc_progress::RtiRefinementRequest request;
  request.progress_mode_active = true;
  request.configured_iterations = 2;
  request.minimum_lateral_bound_reserve_m = 0.0;
  request.minimum_bound_reserve_threshold_m = 0.12;
  request.lateral_defect_threshold_m = 0.08;
  request.heading_defect_threshold_rad = 0.04;
  request.curvature_threshold_radpm = 0.08;
  request.elapsed_ms = 12.0;
  request.refinement_start_deadline_ms = 12.0;
  EXPECT_EQ(
    multi_purpose_mpc_ros::mpcc_progress::resolve_rti_refinement(request),
    RtiRefinementDecision::SkipDeadline);
}

TEST(MpccProgress, KeepsFirstFeasibleSolutionDuringColdEntryLoad)
{
  using multi_purpose_mpc_ros::mpcc_progress::RtiRefinementDecision;
  multi_purpose_mpc_ros::mpcc_progress::RtiRefinementRequest request;
  request.progress_mode_active = true;
  request.configured_iterations = 2;
  request.cold_load_active = true;
  request.minimum_lateral_bound_reserve_m = 0.0;
  request.minimum_bound_reserve_threshold_m = 0.12;
  request.lateral_defect_threshold_m = 0.08;
  request.heading_defect_threshold_rad = 0.04;
  request.curvature_threshold_radpm = 0.08;
  request.elapsed_ms = 2.0;
  request.refinement_start_deadline_ms = 12.0;
  EXPECT_EQ(
    multi_purpose_mpc_ros::mpcc_progress::resolve_rti_refinement(request),
    RtiRefinementDecision::SkipColdLoad);

  request.cold_load_active = false;
  EXPECT_EQ(
    multi_purpose_mpc_ros::mpcc_progress::resolve_rti_refinement(request),
    RtiRefinementDecision::Refine);
}

TEST(MpccProgress, ClassifiesOnlyBoundedEntryOrWallCacheLoadAsCold)
{
  using multi_purpose_mpc_ros::mpcc_progress::RtiColdLoadRequest;
  using multi_purpose_mpc_ros::mpcc_progress::rti_refinement_cold_load_active;
  RtiColdLoadRequest request;
  request.progress_execution_context_active = true;
  request.now_sec = 10.20;
  request.mission_start_sec = 10.0;
  request.cold_entry_skip_sec = 0.30;
  request.wall_cache_miss_skip_threshold = 1U;
  EXPECT_TRUE(rti_refinement_cold_load_active(request));

  request.now_sec = 10.31;
  EXPECT_FALSE(rti_refinement_cold_load_active(request));
  request.wall_cache_miss_count = 1U;
  EXPECT_TRUE(rti_refinement_cold_load_active(request));

  request.progress_execution_context_active = false;
  EXPECT_FALSE(rti_refinement_cold_load_active(request));
}

TEST(MpccProgress, ExtractsBoundedExecutionTrajectoryFromQpPrimal)
{
  // N=2: 3 states x 3 stages followed by 2 inputs x 2 stages.
  Eigen::VectorXd primal(13);
  primal <<
    0.0, 0.0, 10.0,
    0.2, 0.01, 10.5,
    0.4, 0.02, 11.0,
    5.0, 0.0,
    5.0, 0.0;
  const auto result =
    multi_purpose_mpc_ros::mpcc_progress::extract_execution_trajectory(
    primal, 2, {0.5, 1.0}, {-0.5, -0.5}, {0.8, 0.8}, 1e-5);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->lateral_m.size(), 2U);
  EXPECT_DOUBLE_EQ(result->lateral_m[0], 0.2);
  EXPECT_DOUBLE_EQ(result->progress_m[1], 11.0);
  EXPECT_NEAR(result->minimum_lateral_bound_reserve_m, 0.4, 1e-12);
}

TEST(MpccProgress, RejectsExecutionTrajectoryOutsideAppliedBounds)
{
  Eigen::VectorXd primal(13);
  primal <<
    0.0, 0.0, 10.0,
    0.2, 0.01, 10.5,
    0.9, 0.02, 11.0,
    5.0, 0.0,
    5.0, 0.0;
  multi_purpose_mpc_ros::mpcc_progress::ExecutionTrajectoryDiagnostic diagnostic;
  EXPECT_FALSE(multi_purpose_mpc_ros::mpcc_progress::extract_execution_trajectory(
    primal, 2, {0.5, 1.0}, {-0.5, -0.5}, {0.8, 0.8}, 1e-5,
    &diagnostic).has_value());
  EXPECT_EQ(
    diagnostic.rejection,
    multi_purpose_mpc_ros::mpcc_progress::ExecutionTrajectoryRejection::
    LateralOutOfBounds);
  EXPECT_EQ(diagnostic.stage, 1);
}

TEST(MpccProgress, AcceptsExecutionTrajectoryInsideSolverResidualTolerance)
{
  Eigen::VectorXd primal(13);
  primal <<
    0.0, 0.0, 10.0,
    0.2, 0.01, 10.5,
    0.801, 0.02, 10.499,
    5.0, 0.0,
    5.0, 0.0;
  const auto result =
    multi_purpose_mpc_ros::mpcc_progress::extract_execution_trajectory(
    primal, 2, {0.5, 1.0}, {-0.5, -0.5}, {0.8, 0.8}, 0.0011);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->lateral_m.back(), 0.801);
  EXPECT_DOUBLE_EQ(result->minimum_lateral_bound_reserve_m, 0.0);
}

TEST(MpccProgress, SelectsOnlyFeasibleExtendedBranch)
{
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEvaluation;
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchSelectionReason;
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchSelectionRequest;
  ExtendedBranchEvaluation left;
  left.side_sign = 1;
  left.attempted = true;
  left.physical_wall_validation_attempted = true;
  left.physical_wall_validation_passed = false;
  left.failure_reason = "physical execution contract: swept footprint wall violation";
  ExtendedBranchEvaluation right;
  right.side_sign = -1;
  right.attempted = true;
  right.physical_wall_validation_attempted = true;
  right.physical_wall_validation_passed = true;
  right.feasible = true;
  right.objective = 12.0;
  right.minimum_lateral_bound_reserve_m = 0.20;
  const auto result = multi_purpose_mpc_ros::mpcc_progress::select_extended_branch(
    ExtendedBranchSelectionRequest{
      left, right, 0, 1, false, 1.0, 0.05});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.selected_side_sign, -1);
  EXPECT_EQ(result.reason, ExtendedBranchSelectionReason::OnlyRightFeasible);
}

TEST(MpccProgress, UsesPhysicallyValidatedBoundaryBranchWhenNoRobustBranchExists)
{
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEligibility;
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEvaluation;
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchSelectionRequest;
  ExtendedBranchEvaluation right;
  right.side_sign = -1;
  right.attempted = true;
  right.feasible = true;
  right.objective = 12.0;
  right.minimum_lateral_bound_reserve_m = 0.0;
  right.physical_wall_validation_attempted = true;
  right.physical_wall_validation_passed = true;

  const auto result = multi_purpose_mpc_ros::mpcc_progress::select_extended_branch(
    ExtendedBranchSelectionRequest{
      ExtendedBranchEvaluation{}, right, 0, 1, false, 1.0, 0.02});

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.selected_side_sign, -1);
  EXPECT_TRUE(result.physical_boundary_fallback_used);
  EXPECT_EQ(
    result.right_eligibility,
    ExtendedBranchEligibility::PhysicalBoundaryFallback);
}

TEST(MpccProgress, RejectsUncheckedBoundaryBranch)
{
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEligibility;
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEvaluation;
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchSelectionRequest;
  ExtendedBranchEvaluation right;
  right.side_sign = -1;
  right.attempted = true;
  right.feasible = true;
  right.objective = 12.0;
  right.minimum_lateral_bound_reserve_m = 0.0;

  const auto result = multi_purpose_mpc_ros::mpcc_progress::select_extended_branch(
    ExtendedBranchSelectionRequest{
      ExtendedBranchEvaluation{}, right, 0, 1, false, 1.0, 0.02});

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.physical_boundary_fallback_used);
  EXPECT_EQ(
    result.right_eligibility,
    ExtendedBranchEligibility::PhysicalWallUnchecked);
}

TEST(MpccProgress, PrefersRobustBranchOverCheaperBoundaryBranch)
{
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEvaluation;
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchSelectionRequest;
  ExtendedBranchEvaluation left;
  left.side_sign = 1;
  left.attempted = true;
  left.feasible = true;
  left.objective = 20.0;
  left.minimum_lateral_bound_reserve_m = 0.10;
  left.physical_wall_validation_attempted = true;
  left.physical_wall_validation_passed = true;
  ExtendedBranchEvaluation right = left;
  right.side_sign = -1;
  right.objective = 1.0;
  right.minimum_lateral_bound_reserve_m = 0.0;
  right.physical_wall_validation_attempted = true;
  right.physical_wall_validation_passed = true;

  const auto result = multi_purpose_mpc_ros::mpcc_progress::select_extended_branch(
    ExtendedBranchSelectionRequest{left, right, 0, 1, false, 1.0, 0.02});

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.selected_side_sign, 1);
  EXPECT_FALSE(result.physical_boundary_fallback_used);
}

TEST(MpccProgress, HoldsNewEntryWhenDualBranchSelectionIsUnavailable)
{
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEntryAdmissionReason;
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEntryAdmissionRequest;
  const auto result =
    multi_purpose_mpc_ros::mpcc_progress::resolve_extended_branch_entry_admission(
    ExtendedBranchEntryAdmissionRequest{true, true, false, 0, 1});
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.admitted);
  EXPECT_TRUE(result.hold_current_path);
  EXPECT_EQ(
    result.reason,
    ExtendedBranchEntryAdmissionReason::SelectionUnavailable);
}

TEST(MpccProgress, HoldsNewEntryWhenSelectedAndMissionSidesDiffer)
{
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEntryAdmissionReason;
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEntryAdmissionRequest;
  const auto result =
    multi_purpose_mpc_ros::mpcc_progress::resolve_extended_branch_entry_admission(
    ExtendedBranchEntryAdmissionRequest{true, true, true, -1, 1});
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.admitted);
  EXPECT_TRUE(result.hold_current_path);
  EXPECT_EQ(
    result.reason,
    ExtendedBranchEntryAdmissionReason::MissionSideMismatch);
}

TEST(MpccProgress, AdmitsSelectedWallSafeBranchAtNewEntry)
{
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEntryAdmissionReason;
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEntryAdmissionRequest;
  const auto result =
    multi_purpose_mpc_ros::mpcc_progress::resolve_extended_branch_entry_admission(
    ExtendedBranchEntryAdmissionRequest{true, true, true, -1, -1});
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.admitted);
  EXPECT_FALSE(result.hold_current_path);
  EXPECT_EQ(result.reason, ExtendedBranchEntryAdmissionReason::SelectedBranch);
}

TEST(MpccProgress, DoesNotApplyEntryGateToActiveMission)
{
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEntryAdmissionReason;
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEntryAdmissionRequest;
  const auto result =
    multi_purpose_mpc_ros::mpcc_progress::resolve_extended_branch_entry_admission(
    ExtendedBranchEntryAdmissionRequest{true, false, false, 0, 1});
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.admitted);
  EXPECT_FALSE(result.hold_current_path);
  EXPECT_EQ(result.reason, ExtendedBranchEntryAdmissionReason::NotRequired);
}

TEST(MpccProgress, RetainsCurrentExtendedBranchWithinObjectiveHysteresis)
{
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEvaluation;
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchSelectionReason;
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchSelectionRequest;
  ExtendedBranchEvaluation left;
  left.side_sign = 1;
  left.attempted = true;
  left.feasible = true;
  left.objective = 10.0;
  left.minimum_lateral_bound_reserve_m = 0.20;
  left.physical_wall_validation_attempted = true;
  left.physical_wall_validation_passed = true;
  ExtendedBranchEvaluation right = left;
  right.side_sign = -1;
  right.objective = 9.5;
  const auto result = multi_purpose_mpc_ros::mpcc_progress::select_extended_branch(
    ExtendedBranchSelectionRequest{
      left, right, 1, 1, false, 1.0, 0.05});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.selected_side_sign, 1);
  EXPECT_EQ(result.reason, ExtendedBranchSelectionReason::CurrentSideHysteresis);
}

TEST(MpccProgress, KeepsCurrentExtendedBranchAfterNoReturn)
{
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchEvaluation;
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchSelectionReason;
  using multi_purpose_mpc_ros::mpcc_progress::ExtendedBranchSelectionRequest;
  ExtendedBranchEvaluation left;
  left.side_sign = 1;
  left.attempted = true;
  left.feasible = true;
  left.objective = 100.0;
  left.minimum_lateral_bound_reserve_m = 0.20;
  left.physical_wall_validation_attempted = true;
  left.physical_wall_validation_passed = true;
  ExtendedBranchEvaluation right = left;
  right.side_sign = -1;
  right.objective = 1.0;
  const auto result = multi_purpose_mpc_ros::mpcc_progress::select_extended_branch(
    ExtendedBranchSelectionRequest{
      left, right, 1, -1, true, 0.0, 0.05});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.selected_side_sign, 1);
  EXPECT_EQ(result.reason, ExtendedBranchSelectionReason::NoReturnCurrentSide);
}

}  // namespace
