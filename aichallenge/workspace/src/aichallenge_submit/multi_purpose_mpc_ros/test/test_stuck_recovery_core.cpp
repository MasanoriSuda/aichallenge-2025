#include "multi_purpose_mpc_ros/stuck_recovery_core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{

using multi_purpose_mpc_ros::stuck_recovery::CoreConfig;
using multi_purpose_mpc_ros::stuck_recovery::CoreInput;
using multi_purpose_mpc_ros::stuck_recovery::CollisionDeliberateStopOverrideRequest;
using multi_purpose_mpc_ros::stuck_recovery::AdaptiveReverseRetryConfig;
using multi_purpose_mpc_ros::stuck_recovery::AdaptiveReverseRetryTracker;
using multi_purpose_mpc_ros::stuck_recovery::DetectorConfig;
using multi_purpose_mpc_ros::stuck_recovery::DetectorDecision;
using multi_purpose_mpc_ros::stuck_recovery::DetectorInput;
using multi_purpose_mpc_ros::stuck_recovery::ExecutionMode;
using multi_purpose_mpc_ros::stuck_recovery::FaultRetryConfig;
using multi_purpose_mpc_ros::stuck_recovery::FaultRetryGate;
using multi_purpose_mpc_ros::stuck_recovery::FaultRetryInput;
using multi_purpose_mpc_ros::stuck_recovery::Gear;
using multi_purpose_mpc_ros::stuck_recovery::ManeuverDirection;
using multi_purpose_mpc_ros::stuck_recovery::RecoveryAction;
using multi_purpose_mpc_ros::stuck_recovery::RecoveryActionType;
using multi_purpose_mpc_ros::stuck_recovery::RecoveryInput;
using multi_purpose_mpc_ros::stuck_recovery::RecoveryReason;
using multi_purpose_mpc_ros::stuck_recovery::RecoveryState;
using multi_purpose_mpc_ros::stuck_recovery::ReverseDirectionPolicyInput;
using multi_purpose_mpc_ros::stuck_recovery::RecoverySupervisor;
using multi_purpose_mpc_ros::stuck_recovery::RejoinSteeringRequest;
using multi_purpose_mpc_ros::stuck_recovery::ReverseActuationCalibration;
using multi_purpose_mpc_ros::stuck_recovery::StuckDetector;
using multi_purpose_mpc_ros::stuck_recovery::StuckRecoveryCore;
using multi_purpose_mpc_ros::stuck_recovery::StuckRejectReason;
using multi_purpose_mpc_ros::stuck_recovery::StuckVerdict;
using multi_purpose_mpc_ros::stuck_recovery::SupervisorConfig;
using multi_purpose_mpc_ros::stuck_recovery::compute_rejoin_steering_tire_angle;
using multi_purpose_mpc_ros::stuck_recovery::recovery_escape_distance_confirmed;
using multi_purpose_mpc_ros::stuck_recovery::should_override_deliberate_stop_for_collision;
using multi_purpose_mpc_ros::stuck_recovery::solver_fallback_requires_reverse_only;
using multi_purpose_mpc_ros::stuck_recovery::reverse_actuation_calibration_is_valid;
using multi_purpose_mpc_ros::stuck_recovery::reverse_stopping_distance_reserve_m;
using multi_purpose_mpc_ros::stuck_recovery::recovery_candidate_commit_allowed;
using multi_purpose_mpc_ros::stuck_recovery::recovery_reverse_direction_required;
using multi_purpose_mpc_ros::stuck_recovery::recovery_reverse_intent_latch_allowed;
using multi_purpose_mpc_ros::stuck_recovery::source_sample_is_current;
using multi_purpose_mpc_ros::stuck_recovery::source_timestamp_is_monotonic;

TEST(StuckRecoveryRejoinSteering, CombinesCurvatureAndSignedPathErrorFeedback)
{
  RejoinSteeringRequest request;
  request.path_curvature_radpm = 0.1;
  request.wheelbase_m = 1.0;
  request.lateral_error_m = 0.2;
  request.heading_error_rad = -0.05;
  request.lateral_error_gain_rad_per_m = 0.6;
  request.heading_error_gain = 1.2;
  request.max_steering_tire_angle_rad = 0.35;

  const auto steering = compute_rejoin_steering_tire_angle(request);
  ASSERT_TRUE(steering.has_value());
  EXPECT_NEAR(
    steering.value(), std::atan(0.1) - 0.6 * 0.2 - 1.2 * -0.05, 1.0e-12);

  request.path_curvature_radpm = 0.0;
  request.heading_error_rad = 0.0;
  EXPECT_LT(compute_rejoin_steering_tire_angle(request).value(), 0.0);
  request.lateral_error_m = -0.2;
  EXPECT_GT(compute_rejoin_steering_tire_angle(request).value(), 0.0);
}

TEST(StuckRecoveryRejoinSteering, AppliesLimitAndRejectsInvalidInputs)
{
  RejoinSteeringRequest request;
  request.wheelbase_m = 1.0;
  request.lateral_error_m = 10.0;
  request.lateral_error_gain_rad_per_m = 1.0;
  request.max_steering_tire_angle_rad = 0.3;
  ASSERT_TRUE(compute_rejoin_steering_tire_angle(request).has_value());
  EXPECT_DOUBLE_EQ(compute_rejoin_steering_tire_angle(request).value(), -0.3);

  request.wheelbase_m = 0.0;
  EXPECT_FALSE(compute_rejoin_steering_tire_angle(request).has_value());
  request.wheelbase_m = 1.0;
  request.lateral_error_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(compute_rejoin_steering_tire_angle(request).has_value());
}

TEST(StuckRecoveryCollisionStopOverride, RequiresSimulationCollisionAndStoppedEgo)
{
  CollisionDeliberateStopOverrideRequest request;
  request.enabled = true;
  request.simulation_environment = true;
  request.collision_hint = true;
  request.has_front_vehicle = true;
  request.forward_intent = true;
  request.signed_speed_mps = 0.10;
  request.stopped_speed_mps = 0.15;
  EXPECT_TRUE(should_override_deliberate_stop_for_collision(request));

  request.simulation_environment = false;
  EXPECT_FALSE(should_override_deliberate_stop_for_collision(request));
  request.simulation_environment = true;
  request.signed_speed_mps = 0.16;
  EXPECT_FALSE(should_override_deliberate_stop_for_collision(request));
  request.signed_speed_mps = 0.10;
  request.collision_hint = false;
  EXPECT_FALSE(should_override_deliberate_stop_for_collision(request));
}

TEST(StuckRecoveryEscapeConfirmation, AppliesToleranceOnlyWithClearFootprint)
{
  EXPECT_TRUE(recovery_escape_distance_confirmed(true, 1.947, 2.0, 0.10));
  EXPECT_FALSE(recovery_escape_distance_confirmed(true, 1.89, 2.0, 0.10));
  EXPECT_FALSE(recovery_escape_distance_confirmed(false, 2.0, 2.0, 0.10));
  EXPECT_FALSE(recovery_escape_distance_confirmed(true, 1.947, 2.0, -0.10));
}

TEST(StuckRecoveryV2XClockDomain, AcceptsMonotonicSimulationSourceStamps)
{
  const std::optional<double> previous_array_stamp{42.0};
  EXPECT_TRUE(source_timestamp_is_monotonic(43.0, previous_array_stamp));
  EXPECT_TRUE(source_sample_is_current(43.04, 43.0, 1.0));
}

TEST(StuckRecoveryV2XClockDomain, RejectsRollbackFutureAndStaleSourceSamples)
{
  const std::optional<double> previous_array_stamp{43.0};
  EXPECT_FALSE(source_timestamp_is_monotonic(42.9, previous_array_stamp));
  EXPECT_FALSE(source_sample_is_current(43.0, 43.1, 1.0));
  EXPECT_FALSE(source_sample_is_current(43.0, 41.9, 1.0));
}

TEST(StuckRecoveryFaultRetry, RequiresContinuousHealthySimulationWindow)
{
  FaultRetryGate gate(FaultRetryConfig{true, 0.5, 0.2});
  FaultRetryInput input;
  input.simulation_environment = true;
  input.race_started = true;
  input.control_enabled = true;
  input.odometry_fresh_and_finite = true;
  input.command_finite = true;
  input.drive_gear_fresh = true;
  input.boost_inactive = true;
  input.v2x_complete = true;
  input.bounded_maneuver_available = true;
  input.now_sec = 10.0;
  EXPECT_FALSE(gate.update(input));
  for (const double now_sec : {10.1, 10.2, 10.3, 10.4}) {
    input.now_sec = now_sec;
    EXPECT_FALSE(gate.update(input));
  }
  input.now_sec = 10.5;
  EXPECT_TRUE(gate.update(input));
}

TEST(StuckRecoveryFaultRetry, ResetsOnUnsafeConditionOrObservationGap)
{
  FaultRetryGate gate(FaultRetryConfig{true, 0.5, 0.2});
  FaultRetryInput input;
  input.simulation_environment = true;
  input.race_started = true;
  input.control_enabled = true;
  input.odometry_fresh_and_finite = true;
  input.command_finite = true;
  input.drive_gear_fresh = true;
  input.boost_inactive = true;
  input.v2x_complete = true;
  input.bounded_maneuver_available = true;
  input.now_sec = 1.0;
  EXPECT_FALSE(gate.update(input));
  input.v2x_complete = false;
  input.now_sec = 1.1;
  EXPECT_FALSE(gate.update(input));
  input.v2x_complete = true;
  input.now_sec = 1.2;
  EXPECT_FALSE(gate.update(input));
  input.now_sec = 1.6;
  EXPECT_FALSE(gate.update(input));
}

TEST(StuckRecoveryFaultRetry, NeverRetriesOutsideSimulation)
{
  FaultRetryGate gate(FaultRetryConfig{true, 0.0, 0.2});
  FaultRetryInput input;
  input.race_started = true;
  input.control_enabled = true;
  input.odometry_fresh_and_finite = true;
  input.command_finite = true;
  input.drive_gear_fresh = true;
  input.boost_inactive = true;
  input.v2x_complete = true;
  input.bounded_maneuver_available = true;
  EXPECT_FALSE(gate.update(input));
}

TEST(StuckRecoveryAdaptiveReverseRetry, DoublesAfterShortRejoinAndCapsTarget)
{
  AdaptiveReverseRetryTracker tracker(
    AdaptiveReverseRetryConfig{true, 2.0, 4.0, 5.0});

  EXPECT_FALSE(tracker.on_recovery_started());
  EXPECT_DOUBLE_EQ(tracker.target_distance_m(0.8), 0.8);

  tracker.on_rejoin_complete();
  EXPECT_FALSE(tracker.observe_normal_forward_progress(1.0));
  EXPECT_TRUE(tracker.on_recovery_started());
  EXPECT_EQ(tracker.retry_level(), 1U);
  EXPECT_DOUBLE_EQ(tracker.target_distance_m(0.8), 1.6);
  EXPECT_DOUBLE_EQ(tracker.target_distance_m(2.0), 4.0);

  tracker.on_rejoin_complete();
  EXPECT_TRUE(tracker.on_recovery_started());
  EXPECT_EQ(tracker.retry_level(), 2U);
  EXPECT_DOUBLE_EQ(tracker.target_distance_m(0.8), 3.2);

  tracker.on_rejoin_complete();
  EXPECT_TRUE(tracker.on_recovery_started());
  EXPECT_EQ(tracker.retry_level(), 3U);
  EXPECT_DOUBLE_EQ(tracker.target_distance_m(0.8), 4.0);
}

TEST(StuckRecoveryAdaptiveReverseRetry, SustainedForwardProgressResetsSequence)
{
  AdaptiveReverseRetryTracker tracker(
    AdaptiveReverseRetryConfig{true, 2.0, 4.0, 5.0});
  tracker.on_recovery_started();
  tracker.on_rejoin_complete();
  EXPECT_FALSE(tracker.observe_normal_forward_progress(2.0));
  EXPECT_TRUE(tracker.observe_normal_forward_progress(3.0));
  EXPECT_FALSE(tracker.recurrence_window_active());

  EXPECT_FALSE(tracker.on_recovery_started());
  EXPECT_EQ(tracker.retry_level(), 0U);
  EXPECT_DOUBLE_EQ(tracker.target_distance_m(0.8), 0.8);
}

TEST(StuckRecoveryAdaptiveReverseRetry, DisabledTrackerPreservesBaseTarget)
{
  AdaptiveReverseRetryTracker tracker(
    AdaptiveReverseRetryConfig{false, 2.0, 4.0, 5.0});
  tracker.on_rejoin_complete();
  EXPECT_FALSE(tracker.on_recovery_started());
  EXPECT_FALSE(tracker.observe_normal_forward_progress(10.0));
  EXPECT_DOUBLE_EQ(tracker.target_distance_m(7.0), 7.0);
  EXPECT_EQ(tracker.retry_level(), 0U);
}

TEST(StuckRecoveryAdaptiveReverseRetry, RejectsUnsafeConfiguration)
{
  EXPECT_THROW(
    AdaptiveReverseRetryTracker(AdaptiveReverseRetryConfig{true, 1.0, 4.0, 5.0}),
    std::invalid_argument);
  EXPECT_THROW(
    AdaptiveReverseRetryTracker(AdaptiveReverseRetryConfig{true, 2.0, 0.0, 5.0}),
    std::invalid_argument);
  EXPECT_THROW(
    AdaptiveReverseRetryTracker(AdaptiveReverseRetryConfig{true, 2.0, 4.0, 0.0}),
    std::invalid_argument);
}

TEST(StuckRecoveryCandidateCommit, DelaysCommitUntilActuationPath)
{
  EXPECT_FALSE(recovery_candidate_commit_allowed(RecoveryState::SuspectStuck));
  EXPECT_FALSE(recovery_candidate_commit_allowed(RecoveryState::WaitAwsimRecovery));
  EXPECT_FALSE(recovery_candidate_commit_allowed(RecoveryState::CheckClearance));
  EXPECT_FALSE(recovery_candidate_commit_allowed(RecoveryState::WaitForClear));
  EXPECT_FALSE(recovery_candidate_commit_allowed(RecoveryState::SafeStop));

  EXPECT_TRUE(recovery_candidate_commit_allowed(RecoveryState::ShiftToReverse));
  EXPECT_TRUE(recovery_candidate_commit_allowed(RecoveryState::WaitReverseReport));
  EXPECT_TRUE(recovery_candidate_commit_allowed(RecoveryState::ReverseManeuver));
  EXPECT_TRUE(recovery_candidate_commit_allowed(RecoveryState::ForwardManeuver));

  EXPECT_FALSE(recovery_candidate_commit_allowed(RecoveryState::StopAndReassess));
  EXPECT_FALSE(recovery_candidate_commit_allowed(RecoveryState::LowSpeedRejoin));
}

TEST(StuckRecoveryReverseIntent, LatchesDirectionOnlyAfterAwsimSettling)
{
  EXPECT_FALSE(recovery_reverse_intent_latch_allowed(RecoveryState::SuspectStuck));
  EXPECT_FALSE(recovery_reverse_intent_latch_allowed(RecoveryState::WaitAwsimRecovery));
  EXPECT_TRUE(recovery_reverse_intent_latch_allowed(RecoveryState::StopAndConfirm));
  EXPECT_TRUE(recovery_reverse_intent_latch_allowed(RecoveryState::CheckClearance));
  EXPECT_TRUE(recovery_reverse_intent_latch_allowed(RecoveryState::WaitForClear));
  EXPECT_TRUE(recovery_reverse_intent_latch_allowed(RecoveryState::ReverseManeuver));
  EXPECT_FALSE(recovery_reverse_intent_latch_allowed(RecoveryState::ForwardManeuver));
  EXPECT_FALSE(recovery_reverse_intent_latch_allowed(RecoveryState::SafeStop));
}

TEST(StuckRecoveryReverseIntent, RequiresExplicitFailureBeforeForwardFallback)
{
  ReverseDirectionPolicyInput input;
  input.obstacle_reverse_first = true;
  EXPECT_TRUE(recovery_reverse_direction_required(input));

  input.forward_fallback_unlocked = true;
  EXPECT_FALSE(recovery_reverse_direction_required(input));

  input.reverse_intent_latched = true;
  EXPECT_TRUE(recovery_reverse_direction_required(input));
  input.reverse_intent_latched = false;
  input.forward_fallback_unlocked = false;

  input.coordinated_stop_active = true;
  EXPECT_TRUE(recovery_reverse_direction_required(input));
  input.forward_fallback_unlocked = true;
  EXPECT_FALSE(recovery_reverse_direction_required(input));
  input.forward_fallback_unlocked = false;
  input.coordinated_stop_active = false;

  input.solver_reverse_only_candidate = true;
  EXPECT_TRUE(recovery_reverse_direction_required(input));
  input.forward_fallback_unlocked = true;
  EXPECT_TRUE(recovery_reverse_direction_required(input));
  input.forward_fallback_unlocked = false;
  input.solver_reverse_only_candidate = false;

  input.reverse_only_episode = true;
  EXPECT_TRUE(recovery_reverse_direction_required(input));
}

DetectorConfig detector_config()
{
  DetectorConfig config;
  config.stopped_speed_mps = 0.1;
  config.moving_speed_mps = 0.2;
  config.forward_intent_speed_mps = 1.0;
  config.forward_intent_acceleration_mps2 = 0.2;
  config.solver_fallback_duration_sec = 2.0;
  // Most detector tests advance synthetic time in coarse steps. Continuity
  // itself has a dedicated test below; production config uses a much smaller
  // value.
  config.max_observation_gap_sec = 10.0;
  config.stationary_duration_sec = 1.0;
  config.max_pose_displacement_m = 0.1;
  config.max_progress_delta_m = 0.1;
  return config;
}

DetectorInput eligible_detector_input(const double now_sec)
{
  DetectorInput input;
  input.now_sec = now_sec;
  input.race_started = true;
  input.control_enabled = true;
  input.odometry_fresh = true;
  input.requested_forward_speed_mps = 2.0;
  input.wall_evidence = true;
  return input;
}

DetectorDecision confirmed_decision()
{
  DetectorDecision decision;
  decision.verdict = StuckVerdict::Confirmed;
  decision.corroborating_evidence = true;
  decision.forward_intent = true;
  decision.stationary_duration_sec = 2.0;
  return decision;
}

SupervisorConfig supervisor_config()
{
  SupervisorConfig config;
  config.awsim_recovery_wait_sec = 0.0;
  config.stop_speed_mps = 0.05;
  config.stop_confirm_sec = 0.0;
  config.clearance_wait_timeout_sec = 0.5;
  config.gear_report_timeout_sec = 0.5;
  config.gear_command_resend_interval_sec = 0.2;
  config.max_gear_command_requests = 1U;
  config.max_reverse_distance_m = 0.8;
  config.max_reverse_duration_sec = 1.0;
  config.max_reverse_speed_mps = 0.8;
  config.reverse_acceleration_magnitude_mps2 = 0.3;
  config.max_attempts = 1U;
  config.rejoin_speed_limit_mps = 1.0;
  config.max_rejoin_lateral_error_m = 0.5;
  config.max_rejoin_heading_error_rad = 0.35;
  config.rejoin_confirm_sec = 0.2;
  config.rejoin_timeout_sec = 2.0;
  config.cooldown_sec = 0.5;
  return config;
}

RecoveryInput healthy_recovery_input(const double now_sec)
{
  RecoveryInput input;
  input.now_sec = now_sec;
  input.detector = confirmed_decision();
  input.race_active = true;
  input.control_enabled = true;
  input.odometry_valid = true;
  input.solver_healthy = true;
  input.awsim_recovery_settled = true;
  input.awsim_recovery_resolved = true;
  input.reported_gear = Gear::Drive;
  input.gear_report_fresh = true;
  input.maneuver_direction = ManeuverDirection::Reverse;
  input.rear_static_clear = true;
  input.rear_v2x_clear = true;
  input.rear_information_complete = true;
  input.rejoin_safe = true;
  return input;
}

void expect_zero_drive(const RecoveryAction & action)
{
  EXPECT_DOUBLE_EQ(action.acceleration_magnitude_mps2, 0.0);
  EXPECT_DOUBLE_EQ(action.steering_tire_angle_rad, 0.0);
}

void advance_to_clearance_check(
  RecoverySupervisor & supervisor, RecoveryInput & input, double & now_sec)
{
  input.now_sec = now_sec;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::HoldStop);
  EXPECT_EQ(supervisor.state(), RecoveryState::SuspectStuck);

  now_sec += 0.01;
  input.now_sec = now_sec;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::HoldStop);
  EXPECT_EQ(supervisor.state(), RecoveryState::WaitAwsimRecovery);

  now_sec += 0.01;
  input.now_sec = now_sec;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::HoldStop);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopAndConfirm);

  now_sec += 0.01;
  input.now_sec = now_sec;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::HoldStop);
  EXPECT_EQ(supervisor.state(), RecoveryState::CheckClearance);
}

void advance_to_reverse(
  RecoverySupervisor & supervisor, RecoveryInput & input, double & now_sec)
{
  advance_to_clearance_check(supervisor, input, now_sec);

  now_sec += 0.01;
  input.now_sec = now_sec;
  const RecoveryAction request = supervisor.update(input);
  EXPECT_EQ(request.type, RecoveryActionType::RequestReverse);
  EXPECT_EQ(request.requested_gear, Gear::Reverse);
  expect_zero_drive(request);
  EXPECT_EQ(supervisor.state(), RecoveryState::ShiftToReverse);

  now_sec += 0.01;
  input.now_sec = now_sec;
  input.reported_gear = Gear::Reverse;
  input.gear_report_fresh = true;
  const RecoveryAction reverse = supervisor.update(input);
  EXPECT_EQ(reverse.type, RecoveryActionType::ReverseCreep);
  EXPECT_EQ(supervisor.state(), RecoveryState::ReverseManeuver);
}

TEST(RecoverySupervisor, AwsimMotionOnlyResolvesAfterFootprintClears)
{
  RecoverySupervisor still_contacting(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  EXPECT_EQ(still_contacting.update(input).type, RecoveryActionType::HoldStop);
  now += 0.01;
  input.now_sec = now;
  EXPECT_EQ(still_contacting.update(input).type, RecoveryActionType::HoldStop);
  EXPECT_EQ(still_contacting.state(), RecoveryState::WaitAwsimRecovery);

  now += 0.01;
  input.now_sec = now;
  input.detector.verdict = StuckVerdict::Moving;
  input.rejoin_safe = false;
  input.awsim_recovery_resolved = false;
  auto action = still_contacting.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::StopConfirmationPending);
  EXPECT_EQ(still_contacting.state(), RecoveryState::StopAndConfirm);

  RecoverySupervisor cleared(supervisor_config());
  now = 0.0;
  input = healthy_recovery_input(now);
  cleared.update(input);
  now += 0.01;
  input.now_sec = now;
  cleared.update(input);
  now += 0.01;
  input.now_sec = now;
  input.detector.verdict = StuckVerdict::Moving;
  input.rejoin_safe = true;
  input.awsim_recovery_resolved = true;
  action = cleared.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::NormalControl);
  EXPECT_EQ(action.reason, RecoveryReason::AwsimRecoveryResolved);
  EXPECT_EQ(cleared.state(), RecoveryState::Normal);
}

TEST(RecoverySupervisor, EvidenceFreePoseNudgeDoesNotResolveWithoutPathProgress)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  supervisor.update(input);

  now += 0.01;
  input.now_sec = now;
  input.detector.verdict = StuckVerdict::Moving;
  input.rejoin_safe = true;
  input.awsim_recovery_resolved = false;
  const auto action = supervisor.update(input);

  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::StopConfirmationPending);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopAndConfirm);
}

TEST(StuckDetectorConfig, RejectsUnsafeNumericConfiguration)
{
  auto config = detector_config();
  config.stopped_speed_mps = -0.1;
  EXPECT_THROW(StuckDetector{config}, std::invalid_argument);

  config = detector_config();
  config.stationary_duration_sec = std::numeric_limits<double>::infinity();
  EXPECT_THROW(StuckDetector{config}, std::invalid_argument);

  config = detector_config();
  config.solver_fallback_recovery_enabled = true;
  config.solver_fallback_duration_sec = 0.0;
  EXPECT_THROW(StuckDetector{config}, std::invalid_argument);

  config = detector_config();
  config.solver_evidence_free_recovery_enabled = true;
  EXPECT_THROW(StuckDetector{config}, std::invalid_argument);

  config = detector_config();
  config.solver_fallback_recovery_enabled = true;
  config.solver_evidence_free_recovery_enabled = true;
  config.solver_evidence_free_duration_sec =
    config.solver_fallback_duration_sec - 0.01;
  EXPECT_THROW(StuckDetector{config}, std::invalid_argument);

  config = detector_config();
  config.max_observation_gap_sec = 0.0;
  EXPECT_THROW(StuckDetector{config}, std::invalid_argument);

  config = detector_config();
  config.evidence_free_recovery_enabled = true;
  config.evidence_free_duration_sec = config.stationary_duration_sec - 0.01;
  EXPECT_THROW(StuckDetector{config}, std::invalid_argument);

  config = detector_config();
  config.coordinated_stop_recovery_enabled = true;
  config.coordinated_stop_duration_sec = config.stationary_duration_sec - 0.01;
  EXPECT_THROW(StuckDetector{config}, std::invalid_argument);

  config = detector_config();
  config.max_progress_delta_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(StuckDetector{config}, std::invalid_argument);

  config = detector_config();
  config.moving_speed_mps = config.stopped_speed_mps - 0.01;
  EXPECT_THROW(StuckDetector{config}, std::invalid_argument);

  config = detector_config();
  config.forward_intent_speed_mps = 0.0;
  config.forward_intent_acceleration_mps2 = 0.0;
  EXPECT_THROW(StuckDetector{config}, std::invalid_argument);
}

TEST(ReverseActuationCalibration, RequiresOppositeBoundedCommandsAndMeasuredStopping)
{
  ReverseActuationCalibration calibration{-0.3, 0.4, 0.8, 0.1};
  EXPECT_TRUE(reverse_actuation_calibration_is_valid(calibration, -1.6, 1.0));

  calibration.stop_acceleration_mps2 = -0.4;
  EXPECT_FALSE(reverse_actuation_calibration_is_valid(calibration, -1.6, 1.0));
  calibration.stop_acceleration_mps2 = 1.1;
  EXPECT_FALSE(reverse_actuation_calibration_is_valid(calibration, -1.6, 1.0));
  calibration.stop_acceleration_mps2 = 0.4;
  calibration.verified_stop_deceleration_mps2 = 0.0;
  EXPECT_FALSE(reverse_actuation_calibration_is_valid(calibration, -1.6, 1.0));
}

TEST(ReverseActuationCalibration, ReservesBrakingAndLatencyDistance)
{
  const ReverseActuationCalibration calibration{-0.3, 0.4, 2.0, 0.1};
  EXPECT_NEAR(reverse_stopping_distance_reserve_m(calibration, -2.0, 0.025), 1.25, 1e-12);
  EXPECT_NEAR(reverse_stopping_distance_reserve_m(calibration, 2.0, 0.025), 1.25, 1e-12);

  auto invalid = calibration;
  invalid.verified_stop_deceleration_mps2 = 0.0;
  EXPECT_TRUE(std::isinf(reverse_stopping_distance_reserve_m(invalid, 2.0, 0.025)));
}

TEST(StuckDetector, ConfirmsOnlyAfterSustainedNoProgressAndEvidence)
{
  StuckDetector detector(detector_config());
  auto input = eligible_detector_input(10.0);

  auto decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Moving);
  EXPECT_EQ(decision.reject_reason, StuckRejectReason::ObservationWindowIncomplete);

  input.now_sec = 10.99;
  decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Moving);

  input.now_sec = 11.0;
  decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Confirmed);
  EXPECT_EQ(decision.reject_reason, StuckRejectReason::None);
  EXPECT_DOUBLE_EQ(decision.stationary_duration_sec, 1.0);
  EXPECT_TRUE(decision.forward_intent);
  EXPECT_TRUE(decision.corroborating_evidence);
}

TEST(StuckDetector, TreatsOneKilometerPerHourAsStoppedWithObstacleEvidence)
{
  auto config = detector_config();
  config.stopped_speed_mps = 0.28;
  config.moving_speed_mps = 0.35;
  config.stationary_duration_sec = 0.25;
  StuckDetector detector(config);
  auto input = eligible_detector_input(20.0);
  input.signed_speed_mps = 0.28;

  EXPECT_EQ(detector.update(input).verdict, StuckVerdict::Moving);
  input.now_sec = 20.25;
  EXPECT_EQ(detector.update(input).verdict, StuckVerdict::Confirmed);

  detector.reset();
  input.now_sec = 21.0;
  input.signed_speed_mps = 0.281;
  const auto moving = detector.update(input);
  EXPECT_EQ(moving.verdict, StuckVerdict::Moving);
  EXPECT_EQ(moving.reject_reason, StuckRejectReason::VehicleMoving);
}

TEST(StuckDetector, RequiresIndependentCorroboratingEvidence)
{
  StuckDetector detector(detector_config());
  auto input = eligible_detector_input(0.0);
  input.wall_evidence = false;
  input.collision_hint = false;
  detector.update(input);
  input.now_sec = 1.0;

  auto decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Suspected);
  EXPECT_EQ(
    decision.reject_reason, StuckRejectReason::MissingCorroboratingEvidence);

  input.collision_hint = true;
  decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Confirmed);
}

TEST(StuckDetector, EvidenceFreeFallbackRequiresLongHealthyNonDeliberateStall)
{
  auto config = detector_config();
  config.evidence_free_recovery_enabled = true;
  config.evidence_free_duration_sec = 3.0;
  StuckDetector detector(config);
  auto input = eligible_detector_input(0.0);
  input.wall_evidence = false;
  input.collision_hint = false;

  detector.update(input);
  input.now_sec = 1.0;
  auto decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Suspected);
  EXPECT_FALSE(decision.evidence_free_qualified);

  input.now_sec = 2.99;
  decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Suspected);

  input.now_sec = 3.0;
  decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Confirmed);
  EXPECT_FALSE(decision.corroborating_evidence);
  EXPECT_TRUE(decision.evidence_free_qualified);

  detector.reset();
  input.now_sec = 10.0;
  input.solver_fallback = true;
  auto solver_config = config;
  solver_config.solver_fallback_recovery_enabled = true;
  StuckDetector solver_detector(solver_config);
  decision = solver_detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::NotEligible);
  EXPECT_EQ(
    decision.reject_reason,
    StuckRejectReason::SolverFallbackMissingWallEvidence);
  EXPECT_FALSE(decision.evidence_free_qualified);

  detector.reset();
  input.now_sec = 20.0;
  input.solver_fallback = false;
  input.deliberate_stop = true;
  detector.update(input);
  input.now_sec = 24.0;
  decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::NotEligible);
  EXPECT_EQ(decision.reject_reason, StuckRejectReason::DeliberateStop);
}

TEST(StuckDetector, CoordinatesOnlySustainedQualifiedDeliberateStop)
{
  auto config = detector_config();
  config.coordinated_stop_recovery_enabled = true;
  config.coordinated_stop_duration_sec = 3.0;
  StuckDetector detector(config);

  auto input = eligible_detector_input(0.0);
  input.wall_evidence = false;
  input.collision_hint = false;
  input.deliberate_stop = true;
  input.coordinated_stop = true;

  auto decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Moving);
  EXPECT_EQ(decision.reject_reason, StuckRejectReason::ObservationWindowIncomplete);
  EXPECT_FALSE(decision.coordinated_stop_qualified);

  input.now_sec = 2.99;
  decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Moving);
  EXPECT_FALSE(decision.coordinated_stop_qualified);

  input.now_sec = 3.0;
  decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Confirmed);
  EXPECT_FALSE(decision.corroborating_evidence);
  EXPECT_TRUE(decision.coordinated_stop_qualified);
  EXPECT_FALSE(decision.evidence_free_qualified);

  detector.reset();
  input.now_sec = 10.0;
  input.coordinated_stop = false;
  decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::NotEligible);
  EXPECT_EQ(decision.reject_reason, StuckRejectReason::DeliberateStop);
}

TEST(
  StuckDetector,
  QualifiesOnlySustainedSolverFallbackWithCurrentWallEvidence) {
  auto config = detector_config();
  config.solver_fallback_recovery_enabled = true;
  StuckDetector detector(config);
  auto input = eligible_detector_input(10.0);
  input.solver_fallback = true;

  auto decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Moving);
  EXPECT_FALSE(decision.solver_fallback_qualified);

  input.now_sec = 11.99;
  decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Moving);
  EXPECT_EQ(
    decision.reject_reason,
    StuckRejectReason::ObservationWindowIncomplete);
  EXPECT_NEAR(decision.solver_fallback_duration_sec, 1.99, 1e-12);

  input.now_sec = 12.0;
  decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Confirmed);
  EXPECT_TRUE(decision.solver_fallback_qualified);
  EXPECT_DOUBLE_EQ(decision.solver_fallback_duration_sec, 2.0);

  input.now_sec = 12.1;
  input.wall_evidence = false;
  input.collision_hint = true;
  decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::NotEligible);
  EXPECT_EQ(
    decision.reject_reason,
    StuckRejectReason::SolverFallbackMissingWallEvidence);
  EXPECT_FALSE(decision.solver_fallback_qualified);

  input.now_sec = 12.2;
  input.wall_evidence = true;
  decision = detector.update(input);
  EXPECT_EQ(
    decision.reject_reason,
    StuckRejectReason::ObservationWindowIncomplete);
  EXPECT_DOUBLE_EQ(decision.solver_fallback_duration_sec, 0.0);
}

TEST(StuckDetector, QualifiesPersistentWallFreeSolverFallbackAfterLongerWindow)
{
  auto config = detector_config();
  config.solver_fallback_recovery_enabled = true;
  config.solver_evidence_free_recovery_enabled = true;
  config.solver_evidence_free_duration_sec = 3.0;
  StuckDetector detector(config);

  auto input = eligible_detector_input(10.0);
  input.solver_fallback = true;
  input.wall_evidence = false;
  input.collision_hint = false;

  auto decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Moving);
  EXPECT_EQ(decision.reject_reason, StuckRejectReason::ObservationWindowIncomplete);
  EXPECT_FALSE(decision.solver_evidence_free_qualified);

  input.now_sec = 12.99;
  decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Moving);
  EXPECT_FALSE(decision.solver_evidence_free_qualified);

  input.now_sec = 13.0;
  decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Confirmed);
  EXPECT_EQ(decision.reject_reason, StuckRejectReason::None);
  EXPECT_FALSE(decision.corroborating_evidence);
  EXPECT_FALSE(decision.solver_fallback_qualified);
  EXPECT_TRUE(decision.solver_evidence_free_qualified);
  EXPECT_DOUBLE_EQ(decision.stationary_duration_sec, 3.0);
  EXPECT_DOUBLE_EQ(decision.solver_fallback_duration_sec, 3.0);
}

TEST(StuckDetector, SolverRecoveryResetsFallbackQualificationTimer) {
  auto config = detector_config();
  config.solver_fallback_recovery_enabled = true;
  StuckDetector detector(config);
  auto input = eligible_detector_input(0.0);
  input.solver_fallback = true;
  detector.update(input);

  input.now_sec = 1.5;
  EXPECT_FALSE(detector.update(input).solver_fallback_qualified);
  input.now_sec = 1.6;
  input.solver_fallback = false;
  detector.update(input);
  input.now_sec = 1.7;
  input.solver_fallback = true;
  auto decision = detector.update(input);
  EXPECT_DOUBLE_EQ(decision.solver_fallback_duration_sec, 0.0);
  input.now_sec = 3.69;
  EXPECT_FALSE(detector.update(input).solver_fallback_qualified);
  input.now_sec = 3.7;
  EXPECT_TRUE(detector.update(input).solver_fallback_qualified);
}

TEST(StuckDetector, ObservationGapResetsFallbackAndStationaryContinuity) {
  auto config = detector_config();
  config.solver_fallback_recovery_enabled = true;
  config.solver_fallback_duration_sec = 0.2;
  config.stationary_duration_sec = 0.2;
  config.max_observation_gap_sec = 0.15;
  StuckDetector detector(config);
  auto input = eligible_detector_input(0.0);
  input.solver_fallback = true;

  EXPECT_FALSE(detector.update(input).solver_fallback_qualified);
  input.now_sec = 0.1;
  EXPECT_FALSE(detector.update(input).solver_fallback_qualified);

  // An executor/odometry outage must not count toward either duration.
  input.now_sec = 1.0;
  auto decision = detector.update(input);
  EXPECT_FALSE(decision.solver_fallback_qualified);
  EXPECT_DOUBLE_EQ(decision.solver_fallback_duration_sec, 0.0);
  EXPECT_DOUBLE_EQ(decision.stationary_duration_sec, 0.0);

  input.now_sec = 1.1;
  EXPECT_FALSE(detector.update(input).solver_fallback_qualified);
  input.now_sec = 1.21;
  EXPECT_TRUE(detector.update(input).solver_fallback_qualified);
}

TEST(StuckDetector, UsesSpeedHysteresisAndResetsAfterActualMotion) {
  StuckDetector detector(detector_config());
  auto input = eligible_detector_input(0.0);
  detector.update(input);

  input.now_sec = 0.5;
  input.signed_speed_mps = 0.15;
  EXPECT_EQ(detector.update(input).reject_reason, StuckRejectReason::ObservationWindowIncomplete);

  input.now_sec = 1.0;
  EXPECT_EQ(detector.update(input).verdict, StuckVerdict::Confirmed);

  input.now_sec = 1.1;
  input.signed_speed_mps = 0.21;
  auto decision = detector.update(input);
  EXPECT_EQ(decision.verdict, StuckVerdict::Moving);
  EXPECT_EQ(decision.reject_reason, StuckRejectReason::VehicleMoving);

  input.now_sec = 1.2;
  input.signed_speed_mps = 0.0;
  EXPECT_EQ(detector.update(input).reject_reason, StuckRejectReason::ObservationWindowIncomplete);
  input.now_sec = 2.19;
  EXPECT_EQ(detector.update(input).verdict, StuckVerdict::Moving);
  input.now_sec = 2.2;
  EXPECT_EQ(detector.update(input).verdict, StuckVerdict::Confirmed);
}

TEST(StuckDetector, PoseAndUnwrappedPathProgressResetTheObservation)
{
  StuckDetector detector(detector_config());
  auto input = eligible_detector_input(0.0);
  detector.update(input);

  input.now_sec = 0.5;
  input.pose_displacement_m = 0.11;
  auto decision = detector.update(input);
  EXPECT_EQ(decision.reject_reason, StuckRejectReason::PoseProgressing);

  input.now_sec = 0.6;
  input.pose_displacement_m = 0.0;
  detector.update(input);
  input.now_sec = 1.6;
  input.unwrapped_progress_delta_m = -0.11;
  decision = detector.update(input);
  EXPECT_EQ(decision.reject_reason, StuckRejectReason::PathProgressing);

  input.now_sec = 1.7;
  input.unwrapped_progress_delta_m = 0.0;
  EXPECT_EQ(detector.update(input).reject_reason, StuckRejectReason::ObservationWindowIncomplete);
}

TEST(StuckDetector, ExcludesIntentionalAndUnsafeConditions)
{
  const auto check = [](DetectorInput input, const StuckRejectReason expected) {
      StuckDetector detector(detector_config());
      const auto decision = detector.update(input);
      EXPECT_EQ(decision.verdict, StuckVerdict::NotEligible);
      EXPECT_EQ(decision.reject_reason, expected);
    };

  auto input = eligible_detector_input(0.0);
  input.race_started = false;
  check(input, StuckRejectReason::RaceNotStarted);
  input = eligible_detector_input(0.0);
  input.control_enabled = false;
  check(input, StuckRejectReason::ControlDisabled);
  input = eligible_detector_input(0.0);
  input.odometry_fresh = false;
  check(input, StuckRejectReason::OdometryStale);
  input = eligible_detector_input(0.0);
  input.solver_fallback = true;
  check(input, StuckRejectReason::SolverFallback);
  input = eligible_detector_input(0.0);
  input.deliberate_stop = true;
  check(input, StuckRejectReason::DeliberateStop);
  input = eligible_detector_input(0.0);
  input.gear_transition_active = true;
  check(input, StuckRejectReason::GearTransition);
  input = eligible_detector_input(0.0);
  input.awsim_recovery_settling = true;
  check(input, StuckRejectReason::AwsimRecoverySettling);
  input = eligible_detector_input(0.0);
  input.requested_forward_speed_mps = 0.0;
  input.requested_acceleration_mps2 = 0.0;
  check(input, StuckRejectReason::NoForwardIntent);
}

TEST(StuckDetector, AwsimSettlingAndNonMonotonicTimeResetTimer)
{
  StuckDetector detector(detector_config());
  auto input = eligible_detector_input(2.0);
  detector.update(input);
  input.now_sec = 2.5;
  input.awsim_recovery_settling = true;
  EXPECT_EQ(detector.update(input).reject_reason, StuckRejectReason::AwsimRecoverySettling);

  input.now_sec = 3.0;
  input.awsim_recovery_settling = false;
  EXPECT_EQ(detector.update(input).reject_reason, StuckRejectReason::ObservationWindowIncomplete);
  input.now_sec = 2.9;
  EXPECT_EQ(detector.update(input).reject_reason, StuckRejectReason::NonMonotonicTime);
  input.now_sec = 3.9;
  EXPECT_EQ(detector.update(input).reject_reason, StuckRejectReason::ObservationWindowIncomplete);
  input.now_sec = 4.9;
  EXPECT_EQ(detector.update(input).verdict, StuckVerdict::Confirmed);
}

TEST(StuckDetector, RejectsNonFiniteRuntimeInputWithoutThrowing)
{
  StuckDetector detector(detector_config());
  auto input = eligible_detector_input(0.0);
  input.signed_speed_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(detector.update(input).reject_reason, StuckRejectReason::InvalidInput);
  input = eligible_detector_input(1.0);
  input.pose_displacement_m = -0.1;
  EXPECT_EQ(detector.update(input).reject_reason, StuckRejectReason::InvalidInput);
}

TEST(RecoverySupervisorConfig, RejectsNonFiniteOrNegativeConfiguration)
{
  auto config = supervisor_config();
  config.stop_speed_mps = -0.01;
  EXPECT_THROW(RecoverySupervisor{config}, std::invalid_argument);

  config = supervisor_config();
  config.gear_report_timeout_sec = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(RecoverySupervisor{config}, std::invalid_argument);

  config = supervisor_config();
  config.max_reverse_distance_m = -0.1;
  EXPECT_THROW(RecoverySupervisor{config}, std::invalid_argument);

  config = supervisor_config();
  config.max_reverse_speed_mps = -0.1;
  EXPECT_THROW(RecoverySupervisor{config}, std::invalid_argument);

  config = supervisor_config();
  config.reverse_acceleration_magnitude_mps2 = -0.1;
  EXPECT_THROW(RecoverySupervisor{config}, std::invalid_argument);

  config = supervisor_config();
  config.rejoin_timeout_sec = std::numeric_limits<double>::infinity();
  EXPECT_THROW(RecoverySupervisor{config}, std::invalid_argument);

  config = supervisor_config();
  config.rejoin_speed_limit_mps = 0.0;
  EXPECT_THROW(RecoverySupervisor{config}, std::invalid_argument);

  config = supervisor_config();
  config.rejoin_solver_recovery_timeout_sec = -0.1;
  EXPECT_THROW(RecoverySupervisor{config}, std::invalid_argument);

  config = supervisor_config();
  config.safe_stop_clear_confirm_sec = -0.1;
  EXPECT_THROW(RecoverySupervisor{config}, std::invalid_argument);

  config = supervisor_config();
  config.aggressive_retry_delay_sec = -0.1;
  EXPECT_THROW(RecoverySupervisor{config}, std::invalid_argument);
}

TEST(RecoverySupervisor, NeverDrivesBeforeFreshMatchingReverseReport)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_clearance_check(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::RequestReverse);
  EXPECT_EQ(action.requested_gear, Gear::Reverse);
  expect_zero_drive(action);

  now += 0.01;
  input.now_sec = now;
  input.gear_report_fresh = false;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  expect_zero_drive(action);

  now += 0.01;
  input.now_sec = now;
  input.gear_report_fresh = true;
  input.reported_gear = Gear::Drive;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  expect_zero_drive(action);

  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Reverse;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::ReverseCreep);
  EXPECT_DOUBLE_EQ(action.acceleration_magnitude_mps2, 0.3);
  EXPECT_TRUE(action.inhibit_boost);
}

TEST(RecoverySupervisor, GearTimeoutAndInvalidFreshReportLatchSafeStop)
{
  auto timeout_config = supervisor_config();
  timeout_config.gear_report_timeout_sec = 0.1;
  RecoverySupervisor timeout_supervisor(timeout_config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_clearance_check(timeout_supervisor, input, now);
  now += 0.01;
  input.now_sec = now;
  timeout_supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  input.gear_report_fresh = false;
  timeout_supervisor.update(input);
  now += 0.11;
  input.now_sec = now;
  auto action = timeout_supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(action.reason, RecoveryReason::GearReportTimedOut);
  EXPECT_TRUE(timeout_supervisor.safe_stop_latched());
  now += 1.0;
  input.now_sec = now;
  input.reported_gear = Gear::Reverse;
  input.gear_report_fresh = true;
  EXPECT_EQ(timeout_supervisor.update(input).type, RecoveryActionType::SafeStop);

  RecoverySupervisor invalid_supervisor(supervisor_config());
  now = 0.0;
  input = healthy_recovery_input(now);
  advance_to_clearance_check(invalid_supervisor, input, now);
  now += 0.01;
  input.now_sec = now;
  invalid_supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Unknown;
  input.gear_report_fresh = true;
  action = invalid_supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(action.reason, RecoveryReason::GearReportInvalid);
}

TEST(RecoverySupervisor, RearUnknownWaitsAndThenTimesOutWithoutGearRequest)
{
  auto config = supervisor_config();
  config.clearance_wait_timeout_sec = 0.2;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.rear_information_complete = false;
  advance_to_clearance_check(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  auto action = supervisor.update(input);
  EXPECT_EQ(supervisor.state(), RecoveryState::WaitForClear);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::RearInformationIncomplete);
  EXPECT_EQ(action.requested_gear, Gear::NoCommand);

  now += 0.2;
  input.now_sec = now;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(action.reason, RecoveryReason::ClearanceWaitTimedOut);
  EXPECT_EQ(supervisor.attempt_count(), 0U);
}

TEST(RecoverySupervisor, RearClearanceCanRecoverWhileWaiting)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.rear_v2x_clear = false;
  advance_to_clearance_check(supervisor, input, now);
  now += 0.01;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).reason, RecoveryReason::RearVehicleBlocked);
  EXPECT_EQ(supervisor.state(), RecoveryState::WaitForClear);

  now += 0.1;
  input.now_sec = now;
  input.rear_v2x_clear = true;
  const auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::RequestReverse);
  EXPECT_EQ(action.reason, RecoveryReason::ReverseGearRequested);
  EXPECT_EQ(supervisor.state(), RecoveryState::ShiftToReverse);
}

TEST(RecoverySupervisor, ClearanceTimeoutCanResumeOnlyAfterStableClearance)
{
  auto config = supervisor_config();
  config.clearance_wait_timeout_sec = 0.2;
  config.clearance_safe_stop_recovery_enabled = true;
  config.safe_stop_clear_confirm_sec = 0.1;
  config.aggressive_sim_recovery_enabled = true;
  config.aggressive_retry_delay_sec = 0.05;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.rear_v2x_clear = false;
  advance_to_clearance_check(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::HoldStop);
  now += 0.2;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::SafeStop);
  EXPECT_EQ(supervisor.state(), RecoveryState::SafeStop);

  // Even aggressive simulation recovery must not churn a new maneuver while
  // the external V2X blocker is unchanged.
  now += 0.10;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::SafeStop);
  EXPECT_EQ(supervisor.state(), RecoveryState::SafeStop);

  input.rear_v2x_clear = true;
  now += 0.01;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::SafeStop);
  now += 0.09;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::SafeStop);
  now += 0.02;
  input.now_sec = now;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::ClearanceCheck);
  EXPECT_EQ(supervisor.state(), RecoveryState::CheckClearance);

  now += 0.01;
  input.now_sec = now;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::RequestReverse);
}

TEST(RecoverySupervisor, NonClearanceSafeStopRemainsLatchedWhenRecheckEnabled)
{
  auto config = supervisor_config();
  config.clearance_safe_stop_recovery_enabled = true;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_clearance_check(supervisor, input, now);
  now += 0.01;
  input.now_sec = now;
  supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Unknown;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.reason, RecoveryReason::GearReportInvalid);
  EXPECT_EQ(action.type, RecoveryActionType::SafeStop);

  input.reported_gear = Gear::Drive;
  now += 1.0;
  input.now_sec = now;
  action = supervisor.update(input);
  EXPECT_EQ(action.reason, RecoveryReason::GearReportInvalid);
  EXPECT_EQ(action.type, RecoveryActionType::SafeStop);
}

TEST(RecoverySupervisor, ReverseDistanceLimitStopsBeforeRequestingDrive)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);

  now += 0.1;
  input.now_sec = now;
  input.traveled_distance_m = 0.8;
  input.episode_traveled_distance_m = 0.8;
  input.signed_speed_mps = -0.1;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::ReverseDistanceLimit);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopBeforeDrive);
  EXPECT_EQ(action.requested_gear, Gear::NoCommand);

  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = -0.06;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);

  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = 0.0;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::RequestDrive);
  EXPECT_EQ(action.requested_gear, Gear::Drive);
}

TEST(RecoverySupervisor, ReverseSpeedLimitPausesAndThenResumesCreep) {
  for (const double speed_mps : {-0.8, 0.8}) {
    RecoverySupervisor supervisor(supervisor_config());
    double now = 0.0;
    auto input = healthy_recovery_input(now);
    advance_to_reverse(supervisor, input, now);

    now += 0.01;
    input.now_sec = now;
    input.signed_speed_mps = speed_mps;
    auto action = supervisor.update(input);
    EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
    EXPECT_EQ(action.reason, RecoveryReason::ReverseSpeedLimit);
    EXPECT_EQ(supervisor.state(), RecoveryState::ReverseManeuver);

    now += 0.01;
    input.now_sec = now;
    input.signed_speed_mps = 0.79;
    action = supervisor.update(input);
    EXPECT_EQ(action.type, RecoveryActionType::ReverseCreep);
    EXPECT_EQ(supervisor.state(), RecoveryState::ReverseManeuver);
  }

  auto zero_limit_config = supervisor_config();
  zero_limit_config.max_reverse_speed_mps = 0.0;
  RecoverySupervisor zero_limit(zero_limit_config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_clearance_check(zero_limit, input, now);
  now += 0.01;
  input.now_sec = now;
  EXPECT_EQ(zero_limit.update(input).type, RecoveryActionType::RequestReverse);
  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Reverse;
  input.gear_report_fresh = true;
  const auto action = zero_limit.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::ReverseSpeedLimit);
  EXPECT_EQ(zero_limit.state(), RecoveryState::StopBeforeDrive);
}

TEST(RecoverySupervisor, ReverseSpeedBelowLimitContinuesCreep) {
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);
  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = -0.799;
  const auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::ReverseCreep);
  EXPECT_EQ(action.reason, RecoveryReason::ReverseInProgress);
}

TEST(RecoverySupervisor, ContinuousReverseBrakesAndResumesWithoutGearCycle)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = -0.6;
  input.reverse_escape_brake_required = true;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::ReverseEscapeBraking);
  EXPECT_EQ(action.requested_gear, Gear::NoCommand);
  EXPECT_EQ(supervisor.state(), RecoveryState::ReverseManeuver);

  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = -0.2;
  input.reverse_escape_brake_required = false;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::ReverseCreep);
  EXPECT_EQ(action.reason, RecoveryReason::ReverseInProgress);
  EXPECT_EQ(supervisor.state(), RecoveryState::ReverseManeuver);
}

TEST(RecoverySupervisor, ReverseCreepUsesTheSelectedSteeringAngle)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.reverse_steering_tire_angle_rad = -0.25;
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  const auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::ReverseCreep);
  EXPECT_DOUBLE_EQ(action.steering_tire_angle_rad, -0.25);
}

TEST(RecoverySupervisor, ReverseGearConfirmationWaitsForCompleteClearance)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_clearance_check(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::RequestReverse);
  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Reverse;
  input.rear_information_complete = false;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::RearInformationIncomplete);
  EXPECT_EQ(supervisor.state(), RecoveryState::WaitReverseReport);

  now += 0.01;
  input.now_sec = now;
  input.rear_information_complete = true;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::ReverseCreep);
  EXPECT_EQ(supervisor.state(), RecoveryState::ReverseManeuver);
}

TEST(RecoverySupervisor, RearWallUsesBoundedForwardCreepWithoutGearChange)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.maneuver_direction = ManeuverDirection::Forward;
  advance_to_clearance_check(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::ForwardCreep);
  EXPECT_EQ(action.requested_gear, Gear::NoCommand);
  EXPECT_DOUBLE_EQ(action.acceleration_magnitude_mps2, 0.5);
  EXPECT_EQ(supervisor.state(), RecoveryState::ForwardManeuver);

  now += 0.01;
  input.now_sec = now;
  input.traveled_distance_m = 0.31;
  input.recovery_escape_confirmed = true;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::LowSpeedRejoin);
  EXPECT_EQ(action.reason, RecoveryReason::ForwardEscapeConfirmed);
  EXPECT_EQ(supervisor.state(), RecoveryState::LowSpeedRejoin);
}

TEST(RecoverySupervisor, ForwardCreepUsesTheSelectedSteeringAngle)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.maneuver_direction = ManeuverDirection::Forward;
  input.reverse_steering_tire_angle_rad = 0.25;
  advance_to_clearance_check(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  const auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::ForwardCreep);
  EXPECT_DOUBLE_EQ(action.steering_tire_angle_rad, 0.25);
}

TEST(RecoverySupervisor, ForwardHazardStopsAndReassessesInsteadOfLatching)
{
  auto config = supervisor_config();
  config.max_attempts = 2U;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.maneuver_direction = ManeuverDirection::Forward;
  advance_to_clearance_check(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  ASSERT_EQ(supervisor.update(input).type, RecoveryActionType::ForwardCreep);
  ASSERT_EQ(supervisor.state(), RecoveryState::ForwardManeuver);

  now += 0.01;
  input.now_sec = now;
  input.rear_v2x_clear = false;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::ForwardHazardAppeared);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopAndReassess);
  EXPECT_FALSE(supervisor.safe_stop_latched());

  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = 0.0;
  input.rear_v2x_clear = true;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::ClearanceCheck);
  EXPECT_EQ(supervisor.state(), RecoveryState::CheckClearance);

  now += 0.01;
  input.now_sec = now;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::ForwardCreep);
  EXPECT_EQ(supervisor.state(), RecoveryState::ForwardManeuver);
  EXPECT_EQ(supervisor.attempt_count(), 2U);
}

TEST(RecoverySupervisor, ImprovingForwardStepStopsAndReturnsToClearanceSelection)
{
  auto config = supervisor_config();
  config.escape_step_distance_m = 0.2;
  config.max_escape_steps = 2U;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.maneuver_direction = ManeuverDirection::Forward;
  input.stepwise_escape = true;
  input.rejoin_safe = false;
  advance_to_clearance_check(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::ForwardCreep);
  EXPECT_EQ(supervisor.escape_step_count(), 1U);

  now += 0.01;
  input.now_sec = now;
  input.traveled_distance_m = 0.2;
  input.step_contact_improved = true;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::EscapeStepComplete);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopAndReassess);

  now += 0.01;
  input.now_sec = now;
  input.traveled_distance_m = 0.0;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::ClearanceCheck);
  EXPECT_EQ(supervisor.state(), RecoveryState::CheckClearance);
}

TEST(RecoverySupervisor, NonImprovingForwardStepLatchesSafeStop)
{
  auto config = supervisor_config();
  config.escape_step_distance_m = 0.2;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.maneuver_direction = ManeuverDirection::Forward;
  input.stepwise_escape = true;
  advance_to_clearance_check(supervisor, input, now);
  now += 0.01;
  input.now_sec = now;
  supervisor.update(input);

  now += 0.01;
  input.now_sec = now;
  input.traveled_distance_m = 0.2;
  input.step_contact_improved = false;
  const auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(action.reason, RecoveryReason::ContactNotImproving);
  EXPECT_EQ(supervisor.state(), RecoveryState::SafeStop);
}

TEST(RecoverySupervisor, StepwiseForwardDurationLimitStopsAndReassesses)
{
  auto config = supervisor_config();
  config.max_forward_duration_sec = 0.1;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.maneuver_direction = ManeuverDirection::Forward;
  input.stepwise_escape = true;
  advance_to_clearance_check(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::ForwardCreep);

  now += 0.1;
  input.now_sec = now;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::ForwardDurationLimit);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopAndReassess);

  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = 0.0;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::ClearanceCheck);
  EXPECT_EQ(supervisor.state(), RecoveryState::CheckClearance);
}

TEST(RecoverySupervisor, StepwiseForwardCollisionWorseningStopsAndReassesses)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.maneuver_direction = ManeuverDirection::Forward;
  input.stepwise_escape = true;
  advance_to_clearance_check(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::ForwardCreep);

  now += 0.01;
  input.now_sec = now;
  input.collision_worsening = true;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::CollisionWorsening);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopAndReassess);

  now += 0.01;
  input.now_sec = now;
  input.collision_worsening = false;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::ClearanceCheck);
  EXPECT_EQ(supervisor.state(), RecoveryState::CheckClearance);
}

TEST(RecoverySupervisor, ReverseDurationCollisionAndRearHazardAllStopCreep) {
  auto duration_config = supervisor_config();
  duration_config.max_reverse_duration_sec = 0.1;
  RecoverySupervisor duration_supervisor(duration_config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(duration_supervisor, input, now);
  now += 0.1;
  input.now_sec = now;
  auto action = duration_supervisor.update(input);
  EXPECT_EQ(action.reason, RecoveryReason::ReverseDurationLimit);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);

  RecoverySupervisor collision_supervisor(supervisor_config());
  now = 0.0;
  input = healthy_recovery_input(now);
  advance_to_reverse(collision_supervisor, input, now);
  now += 0.01;
  input.now_sec = now;
  input.collision_worsening = true;
  action = collision_supervisor.update(input);
  EXPECT_EQ(action.reason, RecoveryReason::CollisionWorsening);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);

  RecoverySupervisor rear_supervisor(supervisor_config());
  now = 0.0;
  input = healthy_recovery_input(now);
  advance_to_reverse(rear_supervisor, input, now);
  now += 0.01;
  input.now_sec = now;
  input.rear_static_clear = false;
  action = rear_supervisor.update(input);
  EXPECT_EQ(action.reason, RecoveryReason::RearStaticBlocked);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(rear_supervisor.state(), RecoveryState::WaitReverseReport);
}

TEST(RecoverySupervisor, StepwiseCollisionWorseningStopsAndReassesses)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.stepwise_escape = true;
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  input.collision_worsening = true;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::CollisionWorsening);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopBeforeDrive);

  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = 0.0;
  action = supervisor.update(input);
  ASSERT_EQ(action.type, RecoveryActionType::RequestDrive);

  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  input.collision_worsening = false;
  action = supervisor.update(input);
  EXPECT_NE(action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(action.reason, RecoveryReason::ClearanceCheck);
  EXPECT_EQ(supervisor.state(), RecoveryState::CheckClearance);
}

TEST(RecoverySupervisor, StepwiseReverseDurationLimitStopsAndReassesses)
{
  auto config = supervisor_config();
  config.max_reverse_duration_sec = 0.1;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.stepwise_escape = true;
  advance_to_reverse(supervisor, input, now);

  now += 0.1;
  input.now_sec = now;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::ReverseDurationLimit);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopBeforeDrive);

  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = 0.0;
  action = supervisor.update(input);
  ASSERT_EQ(action.type, RecoveryActionType::RequestDrive);

  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  action = supervisor.update(input);
  EXPECT_NE(action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(action.reason, RecoveryReason::ClearanceCheck);
  EXPECT_EQ(supervisor.state(), RecoveryState::CheckClearance);
}

TEST(RecoverySupervisor, NonStepwiseReverseDurationUsesRemainingAttemptForReassessment)
{
  auto config = supervisor_config();
  config.max_reverse_duration_sec = 0.1;
  config.max_attempts = 2U;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);
  ASSERT_EQ(supervisor.attempt_count(), 1U);

  now += 0.1;
  input.now_sec = now;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::ReverseDurationLimit);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopBeforeDrive);
  EXPECT_TRUE(supervisor.drive_report_will_reassess_or_stop());

  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = 0.0;
  action = supervisor.update(input);
  ASSERT_EQ(action.type, RecoveryActionType::RequestDrive);

  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::ClearanceCheck);
  EXPECT_EQ(supervisor.state(), RecoveryState::CheckClearance);

  now += 0.01;
  input.now_sec = now;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::RequestReverse);
  EXPECT_EQ(supervisor.attempt_count(), 2U);
}

TEST(RecoverySupervisor, ReverseManeuverPausesForTransientClearanceLossAndResumes)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.stepwise_escape = true;
  advance_to_reverse(supervisor, input, now);
  const std::size_t attempt_count = supervisor.attempt_count();
  const std::size_t escape_step_count = supervisor.escape_step_count();

  now += 0.01;
  input.now_sec = now;
  input.rear_information_complete = false;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::RearInformationIncomplete);
  EXPECT_EQ(supervisor.state(), RecoveryState::WaitReverseReport);

  now += 0.01;
  input.now_sec = now;
  input.rear_information_complete = true;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::ReverseCreep);
  EXPECT_EQ(action.reason, RecoveryReason::ReverseInProgress);
  EXPECT_EQ(supervisor.state(), RecoveryState::ReverseManeuver);
  EXPECT_EQ(supervisor.attempt_count(), attempt_count);
  EXPECT_EQ(supervisor.escape_step_count(), escape_step_count);
}

TEST(RecoverySupervisor, IncompleteReverseInformationWaitsPastTimeoutAndResumes)
{
  auto config = supervisor_config();
  config.clearance_wait_timeout_sec = 0.1;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  input.rear_information_complete = false;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::RearInformationIncomplete);
  EXPECT_EQ(supervisor.state(), RecoveryState::WaitReverseReport);

  now += 1.0;
  input.now_sec = now;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::RearInformationIncomplete);
  EXPECT_EQ(supervisor.state(), RecoveryState::WaitReverseReport);

  now += 0.01;
  input.now_sec = now;
  input.rear_information_complete = true;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::ReverseCreep);
  EXPECT_EQ(action.reason, RecoveryReason::ReverseInProgress);
  EXPECT_EQ(supervisor.state(), RecoveryState::ReverseManeuver);
}

TEST(RecoverySupervisor, PersistentReverseClearanceLossReturnsToDriveAfterTimeout)
{
  auto config = supervisor_config();
  config.clearance_wait_timeout_sec = 0.1;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  input.rear_v2x_clear = false;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(supervisor.state(), RecoveryState::WaitReverseReport);

  now += 0.1;
  input.now_sec = now;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::RearHazardAppeared);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopBeforeDrive);
}

TEST(RecoverySupervisor, PersistentStepwiseReverseClearanceLossReassessesAfterDrive)
{
  auto config = supervisor_config();
  config.clearance_wait_timeout_sec = 0.1;
  config.max_escape_steps = 2U;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.stepwise_escape = true;
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  input.rear_static_clear = false;
  EXPECT_EQ(supervisor.update(input).reason, RecoveryReason::RearStaticBlocked);
  EXPECT_EQ(supervisor.state(), RecoveryState::WaitReverseReport);

  now += 0.1;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).reason, RecoveryReason::RearHazardAppeared);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopBeforeDrive);
  EXPECT_TRUE(supervisor.drive_report_will_reassess_or_stop());

  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = 0.0;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::RequestDrive);

  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  const auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::ClearanceCheck);
  EXPECT_EQ(supervisor.state(), RecoveryState::CheckClearance);
  EXPECT_EQ(supervisor.escape_step_count(), 1U);
}

TEST(RecoverySupervisor, DriveConfirmationWithoutEscapeLatchesSafeStop)
{
  auto config = supervisor_config();
  config.clearance_wait_timeout_sec = 0.1;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  input.rear_v2x_clear = false;
  supervisor.update(input);
  now += 0.1;
  input.now_sec = now;
  ASSERT_EQ(supervisor.update(input).reason, RecoveryReason::RearHazardAppeared);
  ASSERT_EQ(supervisor.state(), RecoveryState::StopBeforeDrive);

  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = 0.0;
  auto action = supervisor.update(input);
  ASSERT_EQ(action.type, RecoveryActionType::RequestDrive);

  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(action.reason, RecoveryReason::EscapeNotConfirmed);
  EXPECT_EQ(supervisor.state(), RecoveryState::SafeStop);
}

TEST(RecoverySupervisor, LowSpeedRejoinHoldsUntilInformationReturns)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  input.recovery_escape_confirmed = true;
  supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  supervisor.update(input);

  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  input.rear_information_complete = false;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::RearInformationIncomplete);
  EXPECT_EQ(supervisor.state(), RecoveryState::LowSpeedRejoin);

  now += 0.01;
  input.now_sec = now;
  input.rear_information_complete = true;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::LowSpeedRejoin);
  EXPECT_TRUE(action.reset_normal_control);
}

TEST(RecoverySupervisor, LowSpeedRejoinHoldsForSolverRecoveryAndThenResumes)
{
  auto config = supervisor_config();
  config.rejoin_solver_recovery_timeout_sec = 0.2;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  input.recovery_escape_confirmed = true;
  supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  auto action = supervisor.update(input);
  ASSERT_EQ(supervisor.state(), RecoveryState::LowSpeedRejoin);
  ASSERT_EQ(action.type, RecoveryActionType::LowSpeedRejoin);

  now += 0.01;
  input.now_sec = now;
  input.solver_healthy = false;
  action = supervisor.update(input);
  EXPECT_EQ(supervisor.state(), RecoveryState::LowSpeedRejoin);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::SolverRecoveryPending);
  expect_zero_drive(action);

  now += 0.1;
  input.now_sec = now;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::SolverRecoveryPending);

  now += 0.01;
  input.now_sec = now;
  input.solver_healthy = true;
  action = supervisor.update(input);
  EXPECT_EQ(supervisor.state(), RecoveryState::LowSpeedRejoin);
  EXPECT_EQ(action.type, RecoveryActionType::LowSpeedRejoin);
  EXPECT_EQ(action.reason, RecoveryReason::RejoinInProgress);
}

TEST(RecoverySupervisor, LowSpeedRejoinSolverRecoveryTimeoutFailsClosed)
{
  auto config = supervisor_config();
  config.rejoin_solver_recovery_timeout_sec = 0.05;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  input.recovery_escape_confirmed = true;
  supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  supervisor.update(input);
  ASSERT_EQ(supervisor.state(), RecoveryState::LowSpeedRejoin);

  now += 0.01;
  input.now_sec = now;
  input.solver_healthy = false;
  EXPECT_EQ(supervisor.update(input).reason, RecoveryReason::SolverRecoveryPending);

  now += 0.05;
  input.now_sec = now;
  const auto action = supervisor.update(input);
  EXPECT_EQ(supervisor.state(), RecoveryState::SafeStop);
  EXPECT_EQ(action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(action.reason, RecoveryReason::SolverUnsafe);
}

TEST(RecoverySupervisor, AggressiveSimulationRejoinDoesNotDependOnSolverHealth)
{
  auto config = supervisor_config();
  config.aggressive_sim_recovery_enabled = true;
  config.rejoin_solver_recovery_timeout_sec = 0.01;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  input.recovery_escape_confirmed = true;
  supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  ASSERT_EQ(supervisor.update(input).type, RecoveryActionType::LowSpeedRejoin);

  now += 0.02;
  input.now_sec = now;
  input.solver_healthy = false;
  const auto action = supervisor.update(input);
  EXPECT_EQ(supervisor.state(), RecoveryState::LowSpeedRejoin);
  EXPECT_EQ(action.type, RecoveryActionType::LowSpeedRejoin);
  EXPECT_EQ(action.reason, RecoveryReason::RejoinInProgress);
}

TEST(RecoverySupervisor, AggressiveSimulationRetriesRecoverableSafeStopWithFreshBudget)
{
  auto config = supervisor_config();
  config.aggressive_sim_recovery_enabled = true;
  config.aggressive_retry_delay_sec = 0.1;
  config.max_attempts = 0U;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_clearance_check(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  auto action = supervisor.update(input);
  ASSERT_EQ(action.type, RecoveryActionType::SafeStop);
  ASSERT_EQ(action.reason, RecoveryReason::AttemptLimitReached);

  now += 0.05;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::SafeStop);

  now += 0.06;
  input.now_sec = now;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::AggressiveRetry);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopAndConfirm);
  EXPECT_EQ(supervisor.attempt_count(), 0U);
  EXPECT_EQ(supervisor.escape_step_count(), 0U);
}

TEST(RecoverySupervisor, AggressiveSimulationKeepsInvalidInputLatched)
{
  auto config = supervisor_config();
  config.aggressive_sim_recovery_enabled = true;
  config.aggressive_retry_delay_sec = 0.0;
  RecoverySupervisor supervisor(config);
  auto input = healthy_recovery_input(0.0);
  input.lateral_error_m = std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(supervisor.update(input).reason, RecoveryReason::InvalidInput);

  input = healthy_recovery_input(1.0);
  const auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(action.reason, RecoveryReason::InvalidInput);
  EXPECT_EQ(supervisor.state(), RecoveryState::SafeStop);
}

TEST(RecoverySupervisor, BlockedRejoinPathReturnsToBoundedClearanceReassessment)
{
  auto config = supervisor_config();
  config.retry_rejoin_blocked_path = true;
  config.max_escape_steps = 2U;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  input.recovery_escape_confirmed = true;
  supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  input.rejoin_forward_clear = false;

  const auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::RejoinPathBlocked);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopAndConfirm);
}

TEST(RecoverySupervisor, BlockedRejoinPathFailsClosedWhenRetryIsDisabled)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  input.recovery_escape_confirmed = true;
  supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  input.rejoin_forward_clear = false;

  const auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(action.reason, RecoveryReason::RejoinPathBlocked);
  EXPECT_EQ(supervisor.state(), RecoveryState::SafeStop);
}

TEST(RecoverySupervisor, FullRecoveryRequestsResetOnceAndCompletesRejoin)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  input.recovery_escape_confirmed = true;
  EXPECT_EQ(supervisor.update(input).reason, RecoveryReason::ReverseEscapeConfirmed);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopBeforeDrive);

  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = 0.0;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::RequestDrive);
  EXPECT_EQ(supervisor.state(), RecoveryState::ShiftToDrive);

  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  input.gear_report_fresh = true;
  input.lateral_error_m = 1.0;
  input.heading_error_rad = 0.5;
  input.rejoin_steering_tire_angle_rad = -0.17;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::LowSpeedRejoin);
  EXPECT_TRUE(action.reset_normal_control);
  EXPECT_DOUBLE_EQ(action.rejoin_speed_limit_mps, 1.0);
  EXPECT_DOUBLE_EQ(action.steering_tire_angle_rad, -0.17);
  EXPECT_TRUE(action.inhibit_boost);

  now += 0.01;
  input.now_sec = now;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::LowSpeedRejoin);
  EXPECT_FALSE(action.reset_normal_control);

  now += 0.01;
  input.now_sec = now;
  input.lateral_error_m = 0.0;
  input.heading_error_rad = 0.0;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::LowSpeedRejoin);
  now += 0.2;
  input.now_sec = now;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::NormalControl);
  EXPECT_EQ(action.reason, RecoveryReason::RejoinComplete);
  EXPECT_EQ(supervisor.state(), RecoveryState::Normal);
  EXPECT_EQ(supervisor.attempt_count(), 1U);

  now += 0.1;
  input.now_sec = now;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::NormalControl);
  EXPECT_EQ(action.reason, RecoveryReason::CooldownActive);
}

TEST(RecoverySupervisor, CompletedRejoinRestoresBudgetForANewRecoveryEpisode)
{
  auto config = supervisor_config();
  config.max_escape_steps = 1U;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.stepwise_escape = true;
  advance_to_reverse(supervisor, input, now);
  ASSERT_EQ(supervisor.escape_step_count(), 1U);
  ASSERT_EQ(supervisor.attempt_count(), 1U);

  now += 0.01;
  input.now_sec = now;
  input.traveled_distance_m = config.escape_step_distance_m;
  input.step_contact_improved = true;
  input.recovery_escape_confirmed = true;
  EXPECT_EQ(supervisor.update(input).reason, RecoveryReason::EscapeStepComplete);
  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = 0.0;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::RequestDrive);
  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::LowSpeedRejoin);
  now += 0.01;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::LowSpeedRejoin);
  now += config.rejoin_confirm_sec;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).reason, RecoveryReason::RejoinComplete);
  ASSERT_EQ(supervisor.escape_step_count(), 1U);

  // Preserve the completed episode count for its final log. Clear it when a
  // later detector event actually starts another recovery episode.
  now += config.cooldown_sec + 0.01;
  input.now_sec = now;
  input.recovery_escape_confirmed = false;
  EXPECT_EQ(supervisor.update(input).reason, RecoveryReason::StuckConfirmed);
  EXPECT_EQ(supervisor.escape_step_count(), 0U);
  EXPECT_EQ(supervisor.attempt_count(), 0U);

  now += 0.01;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::HoldStop);
  now += 0.01;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::HoldStop);
  now += 0.01;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::HoldStop);
  now += 0.01;
  input.now_sec = now;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::RequestReverse);
  EXPECT_EQ(supervisor.escape_step_count(), 1U);
  EXPECT_EQ(supervisor.attempt_count(), 1U);
}

TEST(RecoverySupervisor, AttemptLimitAndRejoinTimeoutAreLatched)
{
  auto no_attempt_config = supervisor_config();
  no_attempt_config.max_attempts = 0U;
  RecoverySupervisor no_attempt(no_attempt_config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_clearance_check(no_attempt, input, now);
  now += 0.01;
  input.now_sec = now;
  auto action = no_attempt.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(action.reason, RecoveryReason::AttemptLimitReached);

  auto timeout_config = supervisor_config();
  timeout_config.rejoin_timeout_sec = 0.1;
  RecoverySupervisor timeout(timeout_config);
  now = 0.0;
  input = healthy_recovery_input(now);
  advance_to_reverse(timeout, input, now);
  now += 0.01;
  input.now_sec = now;
  input.recovery_escape_confirmed = true;
  timeout.update(input);
  now += 0.01;
  input.now_sec = now;
  timeout.update(input);
  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  input.lateral_error_m = 1.0;
  input.heading_error_rad = 1.0;
  timeout.update(input);
  now += 0.11;
  input.now_sec = now;
  action = timeout.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(action.reason, RecoveryReason::RejoinTimedOut);
}

TEST(RecoverySupervisor, ConfiguredRejoinTimeoutReturnsToBoundedReassessment)
{
  auto config = supervisor_config();
  config.retry_rejoin_timeout = true;
  config.rejoin_timeout_sec = 0.1;
  config.max_attempts = 2U;
  config.max_escape_steps = 2U;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  input.recovery_escape_confirmed = true;
  ASSERT_EQ(supervisor.update(input).reason, RecoveryReason::ReverseEscapeConfirmed);
  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = 0.0;
  ASSERT_EQ(supervisor.update(input).type, RecoveryActionType::RequestDrive);
  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  input.lateral_error_m = 1.0;
  input.heading_error_rad = 1.0;
  ASSERT_EQ(supervisor.update(input).type, RecoveryActionType::LowSpeedRejoin);

  now += 0.11;
  input.now_sec = now;
  auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::RejoinTimedOut);
  EXPECT_EQ(supervisor.state(), RecoveryState::StopAndConfirm);
  EXPECT_FALSE(supervisor.safe_stop_latched());

  now += 0.01;
  input.now_sec = now;
  input.recovery_escape_confirmed = false;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::HoldStop);
  EXPECT_EQ(action.reason, RecoveryReason::ClearanceCheck);
  EXPECT_EQ(supervisor.state(), RecoveryState::CheckClearance);

  now += 0.01;
  input.now_sec = now;
  action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::RequestReverse);
  EXPECT_EQ(supervisor.attempt_count(), 2U);
}

TEST(RecoverySupervisor, RejoinTimeoutStillFailsClosedWithoutRetryBudget)
{
  auto config = supervisor_config();
  config.retry_rejoin_timeout = true;
  config.rejoin_timeout_sec = 0.1;
  config.max_attempts = 1U;
  config.max_escape_steps = 0U;
  RecoverySupervisor supervisor(config);
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);

  now += 0.01;
  input.now_sec = now;
  input.recovery_escape_confirmed = true;
  supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  input.signed_speed_mps = 0.0;
  supervisor.update(input);
  now += 0.01;
  input.now_sec = now;
  input.reported_gear = Gear::Drive;
  input.lateral_error_m = 1.0;
  input.heading_error_rad = 1.0;
  supervisor.update(input);

  now += 0.11;
  input.now_sec = now;
  const auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(action.reason, RecoveryReason::RejoinTimedOut);
  EXPECT_EQ(supervisor.state(), RecoveryState::SafeStop);
}

TEST(RecoverySupervisor, SessionResetClearsLatchedStopTimersAndAttempts)
{
  RecoverySupervisor supervisor(supervisor_config());
  double now = 0.0;
  auto input = healthy_recovery_input(now);
  input.odometry_valid = false;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::NormalControl);
  EXPECT_FALSE(supervisor.safe_stop_latched());

  input = healthy_recovery_input(now);
  advance_to_reverse(supervisor, input, now);
  now += 0.01;
  input.now_sec = now;
  input.odometry_valid = false;
  EXPECT_EQ(supervisor.update(input).type, RecoveryActionType::SafeStop);
  EXPECT_TRUE(supervisor.safe_stop_latched());

  supervisor.reset_session();
  EXPECT_EQ(supervisor.state(), RecoveryState::Normal);
  EXPECT_FALSE(supervisor.safe_stop_latched());
  EXPECT_EQ(supervisor.attempt_count(), 0U);

  input = healthy_recovery_input(0.0);
  input.race_active = false;
  const auto action = supervisor.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::NormalControl);
  EXPECT_EQ(action.reason, RecoveryReason::RaceInactive);
  EXPECT_EQ(supervisor.state(), RecoveryState::Normal);
}

TEST(RecoverySupervisor, RuntimeNonFiniteAndReversedTimeFailSafe)
{
  RecoverySupervisor invalid(supervisor_config());
  auto input = healthy_recovery_input(0.0);
  input.traveled_distance_m = std::numeric_limits<double>::quiet_NaN();
  auto action = invalid.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(action.reason, RecoveryReason::InvalidInput);

  RecoverySupervisor reversed(supervisor_config());
  input = healthy_recovery_input(1.0);
  reversed.update(input);
  input.now_sec = 0.9;
  action = reversed.update(input);
  EXPECT_EQ(action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(action.reason, RecoveryReason::NonMonotonicTime);
}

TEST(StuckRecoveryCore, DefaultsDisabledAndPreservesNormalControl)
{
  StuckRecoveryCore core(CoreConfig{});
  CoreInput input;
  input.detector = eligible_detector_input(0.0);
  const auto output = core.update(input);
  EXPECT_EQ(output.execution_mode, ExecutionMode::Disabled);
  EXPECT_FALSE(output.actuation_allowed);
  EXPECT_FALSE(output.shadow_candidate);
  EXPECT_EQ(output.detector.reject_reason, StuckRejectReason::FeatureDisabled);
  EXPECT_EQ(output.action.type, RecoveryActionType::NormalControl);
  EXPECT_EQ(output.action.requested_gear, Gear::NoCommand);
  EXPECT_FALSE(output.action.inhibit_boost);
}

TEST(StuckRecoveryCore, ShadowAndSimulationGateNeverActuate)
{
  CoreConfig shadow_config;
  shadow_config.enabled = true;
  shadow_config.shadow_mode = true;
  shadow_config.detector = detector_config();
  shadow_config.detector.stationary_duration_sec = 0.0;
  StuckRecoveryCore shadow(shadow_config);
  CoreInput input;
  input.simulation_environment = true;
  input.detector = eligible_detector_input(0.0);
  auto output = shadow.update(input);
  EXPECT_EQ(output.execution_mode, ExecutionMode::Shadow);
  EXPECT_TRUE(output.shadow_candidate);
  EXPECT_FALSE(output.actuation_allowed);
  EXPECT_EQ(output.detector.verdict, StuckVerdict::Confirmed);
  EXPECT_EQ(output.action.type, RecoveryActionType::NormalControl);
  EXPECT_EQ(output.action.reason, RecoveryReason::ShadowMode);
  EXPECT_EQ(shadow.supervisor().state(), RecoveryState::Normal);

  CoreConfig simulation_config = shadow_config;
  simulation_config.shadow_mode = false;
  simulation_config.simulation_only = true;
  StuckRecoveryCore blocked(simulation_config);
  input.simulation_environment = false;
  output = blocked.update(input);
  EXPECT_EQ(output.execution_mode, ExecutionMode::SimulationOnlyBlocked);
  EXPECT_TRUE(output.shadow_candidate);
  EXPECT_FALSE(output.actuation_allowed);
  EXPECT_EQ(output.action.type, RecoveryActionType::NormalControl);
  EXPECT_EQ(output.action.reason, RecoveryReason::SimulationOnlyBlocked);
}

TEST(StuckRecoveryCore, ActiveSimulationCanTakeExclusiveHoldControl)
{
  CoreConfig config;
  config.enabled = true;
  config.shadow_mode = false;
  config.simulation_only = true;
  config.detector = detector_config();
  config.detector.stationary_duration_sec = 0.0;
  config.supervisor = supervisor_config();
  StuckRecoveryCore core(config);

  CoreInput input;
  input.simulation_environment = true;
  input.detector = eligible_detector_input(0.0);
  input.recovery = healthy_recovery_input(0.0);
  const auto output = core.update(input);
  EXPECT_EQ(output.execution_mode, ExecutionMode::Active);
  EXPECT_TRUE(output.actuation_allowed);
  EXPECT_EQ(output.detector.verdict, StuckVerdict::Confirmed);
  EXPECT_EQ(output.state, RecoveryState::SuspectStuck);
  EXPECT_EQ(output.action.type, RecoveryActionType::HoldStop);
  EXPECT_TRUE(output.action.inhibit_boost);
}

TEST(StuckRecoveryCore, DeliberateStopDoesNotBecomeAnImplicitRecoveryHardStop)
{
  CoreConfig config;
  config.enabled = true;
  config.shadow_mode = false;
  config.simulation_only = true;
  config.detector = detector_config();
  config.detector.stationary_duration_sec = 0.0;
  config.supervisor = supervisor_config();
  StuckRecoveryCore core(config);

  CoreInput input;
  input.simulation_environment = true;
  input.detector = eligible_detector_input(0.0);
  input.recovery = healthy_recovery_input(0.0);
  core.update(input);

  input.detector.now_sec = 0.01;
  input.recovery.now_sec = 0.01;
  auto output = core.update(input);
  ASSERT_EQ(output.state, RecoveryState::WaitAwsimRecovery);

  input.detector.now_sec = 0.02;
  input.detector.deliberate_stop = true;
  input.recovery.now_sec = 0.02;
  input.recovery.rejoin_safe = false;
  input.recovery.awsim_recovery_resolved = false;
  output = core.update(input);
  EXPECT_EQ(output.detector.reject_reason, StuckRejectReason::DeliberateStop);
  EXPECT_NE(output.state, RecoveryState::SafeStop);
  EXPECT_EQ(output.state, RecoveryState::StopAndConfirm);
  EXPECT_EQ(output.action.type, RecoveryActionType::HoldStop);
}

TEST(StuckRecoveryCore, AggressiveSimulationModeCannotBeEnabledForNonSimulationCore)
{
  CoreConfig config;
  config.enabled = true;
  config.shadow_mode = false;
  config.simulation_only = false;
  config.detector = detector_config();
  config.supervisor = supervisor_config();
  config.supervisor.aggressive_sim_recovery_enabled = true;
  EXPECT_THROW(StuckRecoveryCore{config}, std::invalid_argument);
}

TEST(StuckRecoveryCore, ExplicitRecoveryHardStopRemainsLatched)
{
  CoreConfig config;
  config.enabled = true;
  config.shadow_mode = false;
  config.simulation_only = true;
  config.detector = detector_config();
  config.detector.stationary_duration_sec = 0.0;
  config.supervisor = supervisor_config();
  StuckRecoveryCore core(config);

  CoreInput input;
  input.simulation_environment = true;
  input.detector = eligible_detector_input(0.0);
  input.recovery = healthy_recovery_input(0.0);
  core.update(input);
  input.detector.now_sec = 0.01;
  input.recovery.now_sec = 0.01;
  ASSERT_EQ(core.update(input).state, RecoveryState::WaitAwsimRecovery);

  input.detector.now_sec = 0.02;
  input.recovery.now_sec = 0.02;
  input.recovery.hard_stop_requested = true;
  const auto output = core.update(input);
  EXPECT_EQ(output.state, RecoveryState::SafeStop);
  EXPECT_EQ(output.action.type, RecoveryActionType::SafeStop);
  EXPECT_EQ(output.state_reason, RecoveryReason::ControlInterrupted);
}

TEST(StuckRecoveryCore, QualifiedSolverFallbackCanEnterExclusiveRecovery) {
  CoreConfig config;
  config.enabled = true;
  config.shadow_mode = false;
  config.simulation_only = true;
  config.detector = detector_config();
  config.detector.solver_fallback_recovery_enabled = true;
  config.detector.stationary_duration_sec = 0.0;
  config.detector.solver_fallback_duration_sec = 0.01;
  config.supervisor = supervisor_config();
  StuckRecoveryCore core(config);

  CoreInput input;
  input.simulation_environment = true;
  input.detector = eligible_detector_input(0.0);
  input.detector.solver_fallback = true;
  input.recovery = healthy_recovery_input(0.0);
  auto output = core.update(input);
  EXPECT_EQ(output.state, RecoveryState::Normal);
  EXPECT_EQ(output.action.type, RecoveryActionType::NormalControl);

  input.detector.now_sec = 0.01;
  input.recovery.now_sec = 0.01;
  output = core.update(input);
  EXPECT_TRUE(output.detector.solver_fallback_qualified);
  EXPECT_EQ(output.state, RecoveryState::SuspectStuck);
  EXPECT_EQ(output.action.type, RecoveryActionType::HoldStop);

  input.detector.now_sec = 0.02;
  input.recovery.now_sec = 0.02;
  output = core.update(input);
  EXPECT_EQ(output.state, RecoveryState::WaitAwsimRecovery);
  EXPECT_EQ(output.action.type, RecoveryActionType::HoldStop);
  EXPECT_NE(output.state_reason, RecoveryReason::SolverUnsafe);
}

TEST(StuckRecoveryCore, QualifiedWallFreeSolverFallbackCanEnterExclusiveRecovery)
{
  CoreConfig config;
  config.enabled = true;
  config.shadow_mode = false;
  config.simulation_only = true;
  config.detector = detector_config();
  config.detector.solver_fallback_recovery_enabled = true;
  config.detector.solver_evidence_free_recovery_enabled = true;
  config.detector.stationary_duration_sec = 0.0;
  config.detector.solver_fallback_duration_sec = 0.01;
  config.detector.solver_evidence_free_duration_sec = 0.01;
  config.supervisor = supervisor_config();
  StuckRecoveryCore core(config);

  CoreInput input;
  input.simulation_environment = true;
  input.detector = eligible_detector_input(0.0);
  input.detector.solver_fallback = true;
  input.detector.wall_evidence = false;
  input.detector.collision_hint = false;
  input.recovery = healthy_recovery_input(0.0);
  auto output = core.update(input);
  EXPECT_EQ(output.state, RecoveryState::Normal);
  EXPECT_EQ(output.action.type, RecoveryActionType::NormalControl);

  input.detector.now_sec = 0.01;
  input.recovery.now_sec = 0.01;
  output = core.update(input);
  EXPECT_TRUE(output.detector.solver_evidence_free_qualified);
  EXPECT_FALSE(output.detector.solver_fallback_qualified);
  EXPECT_EQ(output.state, RecoveryState::SuspectStuck);
  EXPECT_EQ(output.action.type, RecoveryActionType::HoldStop);
}

TEST(StuckRecoveryCore, QualifiedFallbackKeepsIndependentRejoinControl) {
  CoreConfig config;
  config.enabled = true;
  config.shadow_mode = false;
  config.simulation_only = true;
  config.detector = detector_config();
  config.detector.solver_fallback_recovery_enabled = true;
  config.detector.stationary_duration_sec = 0.0;
  config.detector.solver_fallback_duration_sec = 0.01;
  config.supervisor = supervisor_config();
  config.supervisor.rejoin_solver_recovery_timeout_sec = 0.05;
  StuckRecoveryCore core(config);

  CoreInput input;
  input.simulation_environment = true;
  input.detector = eligible_detector_input(0.0);
  input.detector.solver_fallback = true;
  input.recovery = healthy_recovery_input(0.0);
  core.update(input);

  bool drive_confirmation_checked = false;
  double drive_confirmation_sec = 0.0;
  for (std::size_t step = 1U; step < 20U; ++step) {
    const double now_sec = static_cast<double>(step) * 0.01;
    input.detector.now_sec = now_sec;
    input.recovery.now_sec = now_sec;
    input.recovery.recovery_escape_confirmed = false;

    const RecoveryState state_before = core.supervisor().state();
    if (state_before == RecoveryState::ShiftToReverse ||
      state_before == RecoveryState::WaitReverseReport ||
      state_before == RecoveryState::ReverseManeuver ||
      state_before == RecoveryState::StopBeforeDrive)
    {
      input.recovery.reported_gear = Gear::Reverse;
    }
    if (state_before == RecoveryState::ReverseManeuver) {
      input.recovery.recovery_escape_confirmed = true;
    }
    if (state_before == RecoveryState::ShiftToDrive ||
      state_before == RecoveryState::WaitDriveReport)
    {
      input.recovery.reported_gear = Gear::Drive;
      const auto output = core.update(input);
      EXPECT_EQ(output.state, RecoveryState::LowSpeedRejoin);
      EXPECT_EQ(output.action.type, RecoveryActionType::LowSpeedRejoin);
      drive_confirmation_checked = true;
      drive_confirmation_sec = now_sec;
      break;
    }

    const auto output = core.update(input);
    ASSERT_NE(output.state, RecoveryState::SafeStop);
  }
  ASSERT_TRUE(drive_confirmation_checked);

  input.detector.now_sec = drive_confirmation_sec + 0.01;
  input.recovery.now_sec = drive_confirmation_sec + 0.01;
  auto output = core.update(input);
  EXPECT_EQ(output.state, RecoveryState::LowSpeedRejoin);
  EXPECT_EQ(output.action.type, RecoveryActionType::LowSpeedRejoin);
  EXPECT_EQ(output.action.reason, RecoveryReason::RejoinInProgress);
  EXPECT_NE(output.state_reason, RecoveryReason::SolverRecoveryPending);

  input.detector.now_sec = drive_confirmation_sec + 0.02;
  input.detector.solver_fallback = false;
  input.recovery.now_sec = drive_confirmation_sec + 0.02;
  output = core.update(input);
  EXPECT_EQ(output.state, RecoveryState::LowSpeedRejoin);
  EXPECT_EQ(output.action.type, RecoveryActionType::LowSpeedRejoin);
}

TEST(StuckRecoveryCore, PersistentFallbackCanReturnToStepReassessment)
{
  CoreConfig config;
  config.enabled = true;
  config.shadow_mode = false;
  config.simulation_only = true;
  config.detector = detector_config();
  config.detector.solver_fallback_recovery_enabled = true;
  config.detector.stationary_duration_sec = 0.0;
  config.detector.solver_fallback_duration_sec = 0.01;
  config.supervisor = supervisor_config();
  config.supervisor.max_escape_steps = 2U;
  StuckRecoveryCore core(config);

  CoreInput input;
  input.simulation_environment = true;
  input.detector = eligible_detector_input(0.0);
  input.detector.solver_fallback = true;
  input.recovery = healthy_recovery_input(0.0);
  input.recovery.stepwise_escape = true;
  input.recovery.rejoin_safe = false;
  core.update(input);

  bool reassessment_checked = false;
  for (std::size_t step = 1U; step < 20U; ++step) {
    const double now_sec = static_cast<double>(step) * 0.01;
    input.detector.now_sec = now_sec;
    input.recovery.now_sec = now_sec;
    input.recovery.traveled_distance_m = 0.0;
    input.recovery.step_contact_improved = false;

    const RecoveryState state_before = core.supervisor().state();
    if (state_before == RecoveryState::ShiftToReverse ||
      state_before == RecoveryState::WaitReverseReport ||
      state_before == RecoveryState::ReverseManeuver ||
      state_before == RecoveryState::StopBeforeDrive)
    {
      input.recovery.reported_gear = Gear::Reverse;
    }
    if (state_before == RecoveryState::ReverseManeuver) {
      input.recovery.traveled_distance_m = config.supervisor.escape_step_distance_m;
      input.recovery.step_contact_improved = true;
    }
    if (state_before == RecoveryState::ShiftToDrive ||
      state_before == RecoveryState::WaitDriveReport)
    {
      EXPECT_TRUE(core.supervisor().drive_report_will_reassess_or_stop());
      input.recovery.reported_gear = Gear::Drive;
      const auto output = core.update(input);
      EXPECT_NE(output.state, RecoveryState::SafeStop);
      EXPECT_EQ(output.state, RecoveryState::CheckClearance);
      EXPECT_EQ(output.action.type, RecoveryActionType::HoldStop);
      EXPECT_EQ(output.state_reason, RecoveryReason::ClearanceCheck);
      reassessment_checked = true;
      break;
    }

    const auto output = core.update(input);
    ASSERT_NE(output.state, RecoveryState::SafeStop);
  }
  EXPECT_TRUE(reassessment_checked);
}

TEST(StuckRecoveryPolicy, EvidenceFreeSolverFallbackRequiresReverse)
{
  EXPECT_TRUE(solver_fallback_requires_reverse_only(
    true, true, false, 0.2, 1.0));
  EXPECT_FALSE(solver_fallback_requires_reverse_only(
    false, true, false, 2.0, 1.0));
}

TEST(StuckRecoveryPolicy, WallEvidenceAllowsDirectionSelectedEscape)
{
  EXPECT_FALSE(solver_fallback_requires_reverse_only(
    true, true, true, -0.86, 1.0));
  EXPECT_TRUE(solver_fallback_requires_reverse_only(
    true, true, true, -1.01, 1.0));
}

TEST(StuckRecoveryPolicy, InvalidHeadingCannotRelaxEvidenceFreeRecovery)
{
  EXPECT_TRUE(solver_fallback_requires_reverse_only(
    true, true, false, std::numeric_limits<double>::quiet_NaN(), 1.0));
  EXPECT_FALSE(solver_fallback_requires_reverse_only(
    true, false, true, std::numeric_limits<double>::quiet_NaN(), 1.0));
}

TEST(StuckRecoveryCore, ResetSessionClearsDetectorAndSupervisorHistory) {
  CoreConfig config;
  config.enabled = true;
  config.shadow_mode = true;
  config.detector = detector_config();
  StuckRecoveryCore core(config);
  CoreInput input;
  input.simulation_environment = true;
  input.detector = eligible_detector_input(0.0);
  core.update(input);
  input.detector.now_sec = 1.0;
  EXPECT_TRUE(core.update(input).shadow_candidate);

  core.reset_session();
  input.detector.now_sec = 2.0;
  const auto output = core.update(input);
  EXPECT_FALSE(output.shadow_candidate);
  EXPECT_EQ(output.detector.reject_reason, StuckRejectReason::ObservationWindowIncomplete);
  EXPECT_EQ(core.supervisor().attempt_count(), 0U);
}

TEST(StuckRecoveryCore, EnumStringsAreStableAndUnknownSafe)
{
  using multi_purpose_mpc_ros::stuck_recovery::to_string;
  EXPECT_STREQ(to_string(StuckVerdict::Confirmed), "Confirmed");
  EXPECT_STREQ(to_string(StuckRejectReason::DeliberateStop), "deliberate_stop");
  EXPECT_STREQ(to_string(Gear::Reverse), "Reverse");
  EXPECT_STREQ(to_string(RecoveryState::WaitReverseReport), "WAIT_REVERSE_REPORT");
  EXPECT_STREQ(to_string(RecoveryActionType::ReverseCreep), "ReverseCreep");
  EXPECT_STREQ(
    to_string(RecoveryReason::RearInformationIncomplete),
    "rear_information_incomplete");
  EXPECT_STREQ(
    to_string(RecoveryReason::ReverseSpeedLimit),
    "reverse_speed_limit");
  EXPECT_STREQ(
    to_string(RecoveryReason::SolverRecoveryPending),
    "solver_recovery_pending");
  EXPECT_STREQ(to_string(RecoveryReason::AggressiveRetry), "aggressive_retry");
  EXPECT_STREQ(
    to_string(ExecutionMode::SimulationOnlyBlocked),
    "simulation_only_blocked");
  EXPECT_STREQ(to_string(static_cast<RecoveryState>(999)), "UNKNOWN");
  EXPECT_STREQ(to_string(static_cast<RecoveryReason>(999)), "unknown");
}

}  // namespace
