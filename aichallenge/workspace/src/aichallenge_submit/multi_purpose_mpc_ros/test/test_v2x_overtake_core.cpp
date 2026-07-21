#include "multi_purpose_mpc_ros/v2x_overtake_core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace
{

using multi_purpose_mpc_ros::v2x_overtake_core::PassSide;
using multi_purpose_mpc_ros::v2x_overtake_core::LowSpeedPassSideCandidate;
using multi_purpose_mpc_ros::v2x_overtake_core::LowSpeedPassSideRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::LowSpeedBypassCandidateRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::LowSpeedShiftSteeringRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::StoppedVehicleLineOwnershipRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::ContinuityAction;
using multi_purpose_mpc_ros::v2x_overtake_core::ContinuityRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::CoursePoint;
using multi_purpose_mpc_ros::v2x_overtake_core::VehicleRelativeLateralRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::ForwardDistanceRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::FrontDangerAction;
using multi_purpose_mpc_ros::v2x_overtake_core::FrontDangerActionRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::FrontHazardHoldRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::ForwardCourseProjectionRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::FollowSpeedLimitRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::OvertakeSpeedReferenceRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::OvertakeSpeedStage;
using multi_purpose_mpc_ros::v2x_overtake_core::StartGridBreakoutSpeedReferenceRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::ShiftOutCompletionRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::OvertakeFrontCapReleaseRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::PassFrontOverlapExclusionRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::ActivePassGapHoldRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::ActiveLineGapLossHoldRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::ValidatedStartGridBreakoutContinuityRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::OvertakeLateralPlannerOwnershipRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::LiveExecutionCorridorBlockRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::LockedTargetPassSideIntrusionRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::OvertakeLineHeadingReferenceRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::SideOvertakeEntryRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::OvertakeLineHorizonProgressRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::PassSideLateralGoalRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::PassCorridorCenterRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::AdaptiveShiftOutClosingSpeedRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::OvertakeGuardPhaseRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::OvertakeCurveContinuationRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::OuterCurveOvertakeRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::InnerCurveOvertakeRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::ActiveHardCurveContinuationRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::PassCompletionRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::PredictionTimeRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::RecoveryExitReason;
using multi_purpose_mpc_ros::v2x_overtake_core::RecoveryPolicyRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::SolverCooldownRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::SolverFallbackNeutralizationRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::SolverFallbackSteeringRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::SolverReentryGateRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::ReacquireRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::SideSelectionReason;
using multi_purpose_mpc_ros::v2x_overtake_core::SideSelectionRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::SpeedLimitRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::StallWatchdogRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::StartWindowStatus;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_effective_speed_limit;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_follow_speed_limit;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_overtake_speed_reference;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_start_grid_breakout_speed_reference;
using multi_purpose_mpc_ros::v2x_overtake_core::is_shiftout_complete;
using multi_purpose_mpc_ros::v2x_overtake_core::can_release_overtake_front_cap;
using multi_purpose_mpc_ros::v2x_overtake_core::can_exclude_locked_target_from_front_overlap;
using multi_purpose_mpc_ros::v2x_overtake_core::can_hold_active_pass_after_gap_loss;
using multi_purpose_mpc_ros::v2x_overtake_core::can_hold_validated_start_grid_breakout;
using multi_purpose_mpc_ros::v2x_overtake_core::should_resolve_curve_pass_side;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_active_line_gap_loss_hold;
using multi_purpose_mpc_ros::v2x_overtake_core::explicit_overtake_line_owns_lateral_plan;
using multi_purpose_mpc_ros::v2x_overtake_core::GapPlannerStateBoundsRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::should_apply_gap_planner_state_bounds;
using multi_purpose_mpc_ros::v2x_overtake_core::should_block_live_execution_corridor;
using multi_purpose_mpc_ros::v2x_overtake_core::locked_target_intrudes_pass_side;
using multi_purpose_mpc_ros::v2x_overtake_core::can_start_side_overtake;
using multi_purpose_mpc_ros::v2x_overtake_core::side_only_target_requires_follow;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_unlatched_pass_closing_speed;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_overtake_line_horizon_progress;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_overtake_line_heading_reference;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_pass_side_lateral_goal;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_pass_corridor_center;
using multi_purpose_mpc_ros::v2x_overtake_core::has_reached_pass_side_lateral_goal;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_adaptive_shiftout_closing_speed;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_overtake_guard_phase;
using multi_purpose_mpc_ros::v2x_overtake_core::can_continue_overtake_in_soft_curve;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_outer_curve_overtake;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_inner_curve_overtake;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_active_hard_curve_continuation;
using multi_purpose_mpc_ros::v2x_overtake_core::advance_prediction_time;
using multi_purpose_mpc_ros::v2x_overtake_core::project_forward_course_progress;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_vehicle_relative_lateral;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_pass_completion;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_target_continuity;
using multi_purpose_mpc_ros::v2x_overtake_core::can_reacquire_during_return;
using multi_purpose_mpc_ros::v2x_overtake_core::integrate_forward_distance;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_recovery_policy;
using multi_purpose_mpc_ros::v2x_overtake_core::update_stall_watchdog;
using multi_purpose_mpc_ros::v2x_overtake_core::update_front_hazard_hold;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_front_danger_action;
using multi_purpose_mpc_ros::v2x_overtake_core::arm_solver_cooldown;
using multi_purpose_mpc_ros::v2x_overtake_core::is_solver_cooldown_active;
using multi_purpose_mpc_ros::v2x_overtake_core::rate_limit_solver_fallback_steering_toward_neutral;
using multi_purpose_mpc_ros::v2x_overtake_core::should_neutralize_solver_fallback_steering;
using multi_purpose_mpc_ros::v2x_overtake_core::update_solver_reentry_gate;
using multi_purpose_mpc_ros::v2x_overtake_core::select_pass_side;
using multi_purpose_mpc_ros::v2x_overtake_core::is_v2x_behavior_session_active;
using multi_purpose_mpc_ros::v2x_overtake_core::can_start_low_speed_bypass;
using multi_purpose_mpc_ros::v2x_overtake_core::should_yield_overtake_line_to_stopped_bypass;
using multi_purpose_mpc_ros::v2x_overtake_core::select_reachable_low_speed_pass_side;
using multi_purpose_mpc_ros::v2x_overtake_core::has_entered_low_speed_pass_corridor;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_low_speed_pass_velocity;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_low_speed_shift_steering;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_low_speed_direct_control_velocity;
using multi_purpose_mpc_ros::v2x_overtake_core::LowSpeedDirectControlPhase;
using multi_purpose_mpc_ros::v2x_overtake_core::is_low_speed_shift_complete;
using multi_purpose_mpc_ros::v2x_overtake_core::should_begin_low_speed_shift_rejoin;
using multi_purpose_mpc_ros::v2x_overtake_core::should_release_low_speed_shift_control;

SpeedLimitRequest speed_request()
{
  SpeedLimitRequest request;
  request.normal_speed_mps = 20.0;
  request.global_hard_cap_mps = 40.0;
  return request;
}

RecoveryPolicyRequest recovery_request()
{
  RecoveryPolicyRequest request;
  request.configured_velocity_limit_mps = 3.0;
  request.elapsed_sec = 0.5;
  request.traveled_distance_m = 0.5;
  request.target_distance_m = 10.0;
  request.lateral_error_m = 0.6;
  request.lateral_completion_m = 0.2;
  request.stalled_sec = 0.0;
  request.stall_timeout_sec = 1.0;
  request.timeout_sec = 5.0;
  return request;
}

FollowSpeedLimitRequest follow_speed_request()
{
  FollowSpeedLimitRequest request;
  request.enabled = true;
  request.front_distance_m = 5.0;
  request.activation_distance_m = 5.0;
  request.front_speed_mps = 3.0;
  request.moving_front_speed_threshold_mps = 1.0;
  request.moving_front_speed_margin_mps = 0.8;
  request.moving_front_target_distance_m = 2.5;
  request.moving_front_recovery_speed_margin_mps = 0.5;
  request.moving_front_distance_gain = 1.0;
  request.slow_front_distance_limit_mps = 2.3;
  request.slow_front_velocity_cap_mps = 3.0;
  request.maximum_speed_mps = 20.0;
  return request;
}

TEST(V2XOvertakeCoreSpeed, UsesCappedNormalSpeedWithoutStartConfiguration)
{
  auto request = speed_request();
  request.normal_speed_mps = 50.0;
  const auto result = resolve_effective_speed_limit(request);
  EXPECT_DOUBLE_EQ(result.speed_mps, 40.0);
  EXPECT_EQ(result.start_window_status, StartWindowStatus::NotConfigured);
}

TEST(V2XFollowSpeedLimit, ActivatesAtFiveMeterBoundaryForMovingFront)
{
  auto request = follow_speed_request();
  request.front_distance_m = 5.001;
  auto resolution = resolve_follow_speed_limit(request);
  EXPECT_FALSE(resolution.active);
  EXPECT_FALSE(std::isfinite(resolution.speed_limit_mps));

  request.front_distance_m = 5.0;
  resolution = resolve_follow_speed_limit(request);
  EXPECT_TRUE(resolution.active);
  EXPECT_TRUE(resolution.moving_front);
  EXPECT_DOUBLE_EQ(resolution.speed_limit_mps, 3.8);
}

TEST(V2XFollowSpeedLimit, SlowFrontUsesDistanceAndConfiguredCap)
{
  auto request = follow_speed_request();
  request.front_speed_mps = 0.5;
  auto resolution = resolve_follow_speed_limit(request);
  EXPECT_TRUE(resolution.active);
  EXPECT_FALSE(resolution.moving_front);
  EXPECT_DOUBLE_EQ(resolution.speed_limit_mps, 2.3);

  request.slow_front_distance_limit_mps = 4.0;
  resolution = resolve_follow_speed_limit(request);
  EXPECT_DOUBLE_EQ(resolution.speed_limit_mps, 3.0);
}

TEST(V2XFollowSpeedLimit, MovingFrontMarginRecoversCenterDistanceContinuously)
{
  auto request = follow_speed_request();

  request.front_distance_m = 3.0;
  auto resolution = resolve_follow_speed_limit(request);
  ASSERT_TRUE(resolution.active);
  ASSERT_TRUE(resolution.moving_front);
  EXPECT_TRUE(resolution.moving_front_clearance_recovery);
  EXPECT_DOUBLE_EQ(resolution.moving_front_speed_margin_mps, 0.5);
  EXPECT_DOUBLE_EQ(resolution.speed_limit_mps, 3.5);

  request.front_distance_m = 2.5;
  resolution = resolve_follow_speed_limit(request);
  EXPECT_TRUE(resolution.moving_front_clearance_recovery);
  EXPECT_DOUBLE_EQ(resolution.moving_front_speed_margin_mps, 0.0);
  EXPECT_DOUBLE_EQ(resolution.speed_limit_mps, 3.0);

  request.front_distance_m = 2.0;
  resolution = resolve_follow_speed_limit(request);
  EXPECT_TRUE(resolution.moving_front_clearance_recovery);
  EXPECT_DOUBLE_EQ(resolution.moving_front_speed_margin_mps, -0.5);
  EXPECT_DOUBLE_EQ(resolution.speed_limit_mps, 2.5);

  request.front_distance_m = 1.0;
  resolution = resolve_follow_speed_limit(request);
  EXPECT_DOUBLE_EQ(resolution.moving_front_speed_margin_mps, -0.5);
  EXPECT_DOUBLE_EQ(resolution.speed_limit_mps, 2.5);
}

TEST(V2XFollowSpeedLimit, ZeroTargetDistancePreservesFixedMovingFrontMargin)
{
  auto request = follow_speed_request();
  request.front_distance_m = 2.0;
  request.moving_front_target_distance_m = 0.0;
  const auto resolution = resolve_follow_speed_limit(request);
  ASSERT_TRUE(resolution.active);
  EXPECT_FALSE(resolution.moving_front_clearance_recovery);
  EXPECT_DOUBLE_EQ(resolution.moving_front_speed_margin_mps, 0.8);
  EXPECT_DOUBLE_EQ(resolution.speed_limit_mps, 3.8);
}

TEST(V2XFollowSpeedLimit, ZeroDistancePreservesLegacyUnboundedGate)
{
  auto request = follow_speed_request();
  request.activation_distance_m = 0.0;
  request.front_distance_m = 20.0;
  const auto resolution = resolve_follow_speed_limit(request);
  EXPECT_TRUE(resolution.active);
  EXPECT_TRUE(resolution.moving_front);
  EXPECT_DOUBLE_EQ(resolution.speed_limit_mps, 3.8);
}

TEST(V2XFollowSpeedLimit, DisableSuppressAndInvalidObservationRemainInactive)
{
  auto request = follow_speed_request();
  request.enabled = false;
  EXPECT_FALSE(resolve_follow_speed_limit(request).active);

  request.enabled = true;
  request.suppressed = true;
  EXPECT_FALSE(resolve_follow_speed_limit(request).active);

  request.suppressed = false;
  request.front_distance_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(resolve_follow_speed_limit(request).active);
  request.front_distance_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(resolve_follow_speed_limit(request).active);
}

TEST(V2XFollowSpeedLimit, RejectsInvalidConfiguration)
{
  auto request = follow_speed_request();
  request.activation_distance_m = -0.1;
  EXPECT_THROW(resolve_follow_speed_limit(request), std::invalid_argument);

  request = follow_speed_request();
  request.moving_front_speed_margin_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(resolve_follow_speed_limit(request), std::invalid_argument);

  request = follow_speed_request();
  request.moving_front_distance_gain = -0.1;
  EXPECT_THROW(resolve_follow_speed_limit(request), std::invalid_argument);
}

TEST(V2XFrontHazardHold, HoldsAcrossDropoutAndRefreshesDeadline)
{
  FrontHazardHoldRequest request;
  request.enabled = true;
  request.hazard_observed = true;
  request.now_sec = 10.0;
  request.current_until_sec = 10.0;
  request.hold_sec = 1.0;
  auto result = update_front_hazard_hold(request);
  ASSERT_TRUE(result.active);
  EXPECT_DOUBLE_EQ(result.until_sec, 11.0);

  request.hazard_observed = false;
  request.now_sec = 10.6;
  request.current_until_sec = result.until_sec;
  result = update_front_hazard_hold(request);
  ASSERT_TRUE(result.active);
  EXPECT_NEAR(result.remaining_sec, 0.4, 1e-12);

  request.hazard_observed = true;
  result = update_front_hazard_hold(request);
  ASSERT_TRUE(result.active);
  EXPECT_DOUBLE_EQ(result.until_sec, 11.6);
}

TEST(V2XFrontHazardHold, RearClearAndExpiryReleaseImmediately)
{
  FrontHazardHoldRequest request{true, false, true, 10.2, 11.0, 1.0};
  auto result = update_front_hazard_hold(request);
  EXPECT_FALSE(result.active);
  EXPECT_DOUBLE_EQ(result.until_sec, 10.2);

  request.target_rear_clear = false;
  request.now_sec = 11.0;
  request.current_until_sec = 11.0;
  result = update_front_hazard_hold(request);
  EXPECT_FALSE(result.active);
  EXPECT_DOUBLE_EQ(result.remaining_sec, 0.0);
}

TEST(V2XFrontHazardHold, SafeMovingTargetObservationReleasesImmediately)
{
  FrontHazardHoldRequest request{true, false, false, 10.2, 11.0, 1.0};
  request.target_observed_safe = true;
  const auto result = update_front_hazard_hold(request);
  EXPECT_FALSE(result.active);
  EXPECT_DOUBLE_EQ(result.until_sec, 10.2);
  EXPECT_DOUBLE_EQ(result.remaining_sec, 0.0);
}

TEST(V2XFrontHazardHold, RejectsInvalidClockAndDuration)
{
  FrontHazardHoldRequest request{true, false, false, 1.0, 1.0, -0.1};
  EXPECT_THROW(update_front_hazard_hold(request), std::invalid_argument);
  request.hold_sec = 1.0;
  request.now_sec = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(update_front_hazard_hold(request), std::invalid_argument);
}

TEST(V2XFrontDangerAction, MovingFrontUsesRelativeSpeedLimit)
{
  FrontDangerActionRequest request;
  request.inside_stopping_distance = true;
  request.front_speed_mps = 3.0;
  request.moving_front_speed_threshold_mps = 1.0;
  EXPECT_EQ(resolve_front_danger_action(request), FrontDangerAction::RelativeSpeedLimit);
}

TEST(V2XFrontDangerAction, MovingFrontInsideHardCenterDistanceUsesSafetyBrake)
{
  FrontDangerActionRequest request;
  request.inside_stopping_distance = true;
  request.front_speed_mps = 3.0;
  request.moving_front_speed_threshold_mps = 1.0;
  request.front_distance_m = 2.0;
  request.moving_front_hard_distance_m = 2.0;
  EXPECT_EQ(resolve_front_danger_action(request), FrontDangerAction::SafetyBrake);

  request.front_distance_m = 2.001;
  EXPECT_EQ(resolve_front_danger_action(request), FrontDangerAction::RelativeSpeedLimit);

  request.inside_stopping_distance = false;
  request.front_distance_m = 1.9;
  EXPECT_EQ(resolve_front_danger_action(request), FrontDangerAction::SafetyBrake);
}

TEST(V2XFrontDangerAction, EmergencyAndStoppedFrontKeepSafetyBrake)
{
  FrontDangerActionRequest request;
  request.inside_stopping_distance = true;
  request.front_speed_mps = 0.2;
  request.moving_front_speed_threshold_mps = 1.0;
  EXPECT_EQ(resolve_front_danger_action(request), FrontDangerAction::SafetyBrake);

  request.front_speed_mps = 5.0;
  request.emergency_brake = true;
  EXPECT_EQ(resolve_front_danger_action(request), FrontDangerAction::SafetyBrake);
}

TEST(V2XFrontDangerAction, ClearGeometryDoesNotLimitAndInvalidTargetFailsClosed)
{
  FrontDangerActionRequest request;
  request.moving_front_speed_threshold_mps = 1.0;
  EXPECT_EQ(resolve_front_danger_action(request), FrontDangerAction::None);

  request.inside_stopping_distance = true;
  request.front_speed_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(resolve_front_danger_action(request), FrontDangerAction::SafetyBrake);

  request.moving_front_speed_threshold_mps = -0.1;
  EXPECT_THROW(resolve_front_danger_action(request), std::invalid_argument);
}

TEST(V2XOvertakeCoreSpeed, StartWindowMayExceedNormalButNotGlobalHardCap)
{
  auto request = speed_request();
  request.start_speed_mps = 45.0;
  request.start_window_duration_sec = 15.0;
  request.elapsed_since_start_sec = 2.0;
  const auto result = resolve_effective_speed_limit(request);
  EXPECT_DOUBLE_EQ(result.speed_mps, 40.0);
  EXPECT_EQ(result.start_window_status, StartWindowStatus::Applied);
}

TEST(V2XOvertakeCoreSpeed, StartWindowExpiresAtDurationBoundary)
{
  auto request = speed_request();
  request.start_speed_mps = 37.0;
  request.start_window_duration_sec = 15.0;
  request.elapsed_since_start_sec = 15.0;
  const auto result = resolve_effective_speed_limit(request);
  EXPECT_DOUBLE_EQ(result.speed_mps, 20.0);
  EXPECT_EQ(result.start_window_status, StartWindowStatus::Expired);
}

TEST(V2XOvertakeCoreSpeed, MissingStartEpochFallsBackToNormalSpeed)
{
  auto request = speed_request();
  request.start_speed_mps = 37.0;
  request.start_window_duration_sec = 15.0;
  request.elapsed_since_start_sec = std::nullopt;
  const auto result = resolve_effective_speed_limit(request);
  EXPECT_DOUBLE_EQ(result.speed_mps, 20.0);
  EXPECT_EQ(result.start_window_status, StartWindowStatus::AwaitingStart);
}

TEST(V2XOvertakeCoreSpeed, InvalidElapsedFallsBackToNormalSpeed)
{
  auto request = speed_request();
  request.start_speed_mps = 37.0;
  request.start_window_duration_sec = 15.0;

  request.elapsed_since_start_sec = -0.001;
  auto result = resolve_effective_speed_limit(request);
  EXPECT_DOUBLE_EQ(result.speed_mps, 20.0);
  EXPECT_EQ(result.start_window_status, StartWindowStatus::InvalidElapsed);

  request.elapsed_since_start_sec = std::numeric_limits<double>::quiet_NaN();
  result = resolve_effective_speed_limit(request);
  EXPECT_DOUBLE_EQ(result.speed_mps, 20.0);
  EXPECT_EQ(result.start_window_status, StartWindowStatus::InvalidElapsed);
}

TEST(V2XOvertakeCoreSpeed, RejectsInvalidSpeedPolicyConfiguration)
{
  auto request = speed_request();
  request.global_hard_cap_mps = -1.0;
  EXPECT_THROW(resolve_effective_speed_limit(request), std::invalid_argument);

  request = speed_request();
  request.start_speed_mps = std::numeric_limits<double>::infinity();
  request.start_window_duration_sec = 15.0;
  EXPECT_THROW(resolve_effective_speed_limit(request), std::invalid_argument);
}

TEST(V2XOvertakeCoreSpeed, CapsShiftOutButReleasesFrontCapInPass)
{
  OvertakeSpeedReferenceRequest request;
  request.base_reference_speed_mps = 11.0;
  request.hard_cap_mps = 11.1;
  request.front_speed_mps = 8.0;
  request.entry_speed_mps = 8.5;
  request.shiftout_max_closing_speed_mps = 1.0;

  auto result = resolve_overtake_speed_reference(request);
  EXPECT_DOUBLE_EQ(result.reference_speed_mps, 9.0);
  EXPECT_TRUE(result.front_cap_applied);

  request.stage = OvertakeSpeedStage::Pass;
  result = resolve_overtake_speed_reference(request);
  EXPECT_DOUBLE_EQ(result.reference_speed_mps, 11.0);
  EXPECT_FALSE(result.front_cap_applied);
}

TEST(V2XOvertakeCoreSpeed, ValidatedStartGridBreakoutReleasesRaceReferenceAtEntry)
{
  StartGridBreakoutSpeedReferenceRequest request;
  request.base_reference_speed_mps = 10.28;
  request.hard_cap_mps = 10.28;
  request.front_speed_mps = 3.08;
  request.entry_speed_mps = 3.66;
  request.shiftout_max_closing_speed_mps = 1.2;

  auto result = resolve_start_grid_breakout_speed_reference(request);
  EXPECT_DOUBLE_EQ(result.reference_speed_mps, 4.28);
  EXPECT_TRUE(result.front_cap_applied);

  request.validated_breakout = true;
  result = resolve_start_grid_breakout_speed_reference(request);
  EXPECT_DOUBLE_EQ(result.reference_speed_mps, 10.28);
  EXPECT_FALSE(result.front_cap_applied);
}

TEST(V2XOvertakeCoreSpeed, RequiresDistanceAndLateralCompletionBeforePass)
{
  ShiftOutCompletionRequest request;
  request.phase_hold_elapsed = true;
  request.traveled_distance_m = 8.0;
  request.required_distance_m = 8.0;
  request.current_lateral_m = 0.73;
  request.target_lateral_m = 1.20;
  request.lateral_tolerance_m = 0.30;
  request.pass_side_sign = 1;
  EXPECT_FALSE(is_shiftout_complete(request));

  request.current_lateral_m = 0.91;
  EXPECT_TRUE(is_shiftout_complete(request));

  request.traveled_distance_m = 7.99;
  EXPECT_FALSE(is_shiftout_complete(request));
  request.traveled_distance_m = 8.0;
  request.phase_hold_elapsed = false;
  EXPECT_FALSE(is_shiftout_complete(request));
}

TEST(V2XOvertakeCoreSpeed, TreatsPassSideOvershootAsLateralCompletion)
{
  EXPECT_TRUE(has_reached_pass_side_lateral_goal(1.82, 1.20, 0.30, 1));
  EXPECT_FALSE(has_reached_pass_side_lateral_goal(0.73, 1.20, 0.30, 1));
  EXPECT_TRUE(has_reached_pass_side_lateral_goal(-1.82, -1.20, 0.30, -1));
  EXPECT_FALSE(has_reached_pass_side_lateral_goal(-0.73, -1.20, 0.30, -1));
  EXPECT_FALSE(has_reached_pass_side_lateral_goal(1.82, 1.20, 0.30, 0));
}

TEST(V2XOvertakeCoreSpeed, ReleasesFrontCapOnlyAfterTargetIsNoLongerAhead)
{
  OvertakeFrontCapReleaseRequest request;
  request.pass_phase = true;
  request.lateral_complete = true;
  request.target_seen = true;
  request.target_longitudinal_m = 0.01;
  EXPECT_FALSE(can_release_overtake_front_cap(request));

  request.target_longitudinal_m = 0.0;
  EXPECT_TRUE(can_release_overtake_front_cap(request));
  request.target_seen = false;
  EXPECT_FALSE(can_release_overtake_front_cap(request));
  request.target_seen = true;
  request.lateral_complete = false;
  EXPECT_FALSE(can_release_overtake_front_cap(request));
  request.lateral_complete = true;
  request.pass_phase = false;
  EXPECT_FALSE(can_release_overtake_front_cap(request));
}

TEST(V2XOvertakeCoreSpeed, ExcludesOnlyLaterallyClearLockedTargetDuringPass)
{
  PassFrontOverlapExclusionRequest request;
  request.pass_phase = true;
  request.locked_target = true;
  request.relative_lateral_m = 1.39;
  request.required_lateral_clearance_m = 1.20;
  EXPECT_TRUE(can_exclude_locked_target_from_front_overlap(request));

  request.relative_lateral_m = 1.19;
  EXPECT_FALSE(can_exclude_locked_target_from_front_overlap(request));
  request.already_latched = true;
  EXPECT_TRUE(can_exclude_locked_target_from_front_overlap(request));

  request.relative_lateral_m = 1.39;
  request.pass_phase = false;
  EXPECT_FALSE(can_exclude_locked_target_from_front_overlap(request));
  request.pass_phase = true;
  request.locked_target = false;
  EXPECT_FALSE(can_exclude_locked_target_from_front_overlap(request));
}

TEST(V2XOvertakeCoreSpeed, KeepsLineGoalAndFrontBrakeExclusionIndependent)
{
  PassSideLateralGoalRequest goal_request;
  goal_request.pass_side_sign = 1;
  goal_request.base_lateral_offset_m = 1.2;
  goal_request.target_lateral_m = 0.0;
  goal_request.minimum_separation_m = 0.75;
  EXPECT_DOUBLE_EQ(resolve_pass_side_lateral_goal(goal_request), 1.2);

  PassFrontOverlapExclusionRequest exclusion_request;
  exclusion_request.pass_phase = true;
  exclusion_request.locked_target = true;
  exclusion_request.relative_lateral_m = 1.0;
  exclusion_request.required_lateral_clearance_m = 1.15;
  EXPECT_FALSE(can_exclude_locked_target_from_front_overlap(exclusion_request));
  exclusion_request.relative_lateral_m = 1.15;
  EXPECT_TRUE(can_exclude_locked_target_from_front_overlap(exclusion_request));
}

TEST(V2XOvertakeCoreSpeed, HoldsOnlyCommittedActivePassAfterGapLoss)
{
  ActivePassGapHoldRequest request;
  request.pass_phase = true;
  request.lateral_clearance_latched = true;
  request.locked_target_seen = true;
  EXPECT_TRUE(can_hold_active_pass_after_gap_loss(request));

  request.pass_phase = false;
  EXPECT_FALSE(can_hold_active_pass_after_gap_loss(request));
  request.pass_phase = true;
  request.lateral_clearance_latched = false;
  EXPECT_FALSE(can_hold_active_pass_after_gap_loss(request));
  request.lateral_clearance_latched = true;
  request.locked_target_seen = false;
  EXPECT_FALSE(can_hold_active_pass_after_gap_loss(request));
  request.locked_target_seen = true;
  request.locked_target_position_jump = true;
  EXPECT_FALSE(can_hold_active_pass_after_gap_loss(request));
}

TEST(V2XOvertakeCoreSpeed, HoldsValidatedStartGridBreakoutAcrossGapReevaluation)
{
  ValidatedStartGridBreakoutContinuityRequest request;
  request.continuing_breakout = true;
  request.active_line = true;
  request.target_matches = true;
  request.locked_target_seen = true;
  EXPECT_TRUE(can_hold_validated_start_grid_breakout(request));

  request.target_matches = false;
  EXPECT_FALSE(can_hold_validated_start_grid_breakout(request));
  request.target_matches = true;
  request.locked_target_position_jump = true;
  EXPECT_FALSE(can_hold_validated_start_grid_breakout(request));
  request.locked_target_position_jump = false;
  request.explicit_forbidden_wp = true;
  EXPECT_FALSE(can_hold_validated_start_grid_breakout(request));
  request.explicit_forbidden_wp = false;
  request.active_line = false;
  EXPECT_FALSE(can_hold_validated_start_grid_breakout(request));
}

TEST(V2XOvertakeCoreGeometry, UsesEgoRelativeLateralForCommonCourseOverlap)
{
  VehicleRelativeLateralRequest request;
  request.course_projection_used = true;
  request.vehicle_course_lateral_m = -1.64;
  request.ego_course_lateral_m = 1.38;
  request.local_relative_lateral_m = -0.2;
  EXPECT_NEAR(resolve_vehicle_relative_lateral(request), -3.02, 1.0e-9);

  request.ego_course_lateral_m = -1.20;
  EXPECT_NEAR(resolve_vehicle_relative_lateral(request), -0.44, 1.0e-9);

  request.course_projection_used = false;
  EXPECT_DOUBLE_EQ(resolve_vehicle_relative_lateral(request), -0.2);
}

TEST(V2XOvertakeCoreSpeed, ResolvesPassSideForSoftAndHardCurvesExceptExplicitForbiddenWp)
{
  EXPECT_TRUE(should_resolve_curve_pass_side(true, false, false));
  EXPECT_TRUE(should_resolve_curve_pass_side(false, true, false));
  EXPECT_TRUE(should_resolve_curve_pass_side(true, true, false));
  EXPECT_FALSE(should_resolve_curve_pass_side(false, false, false));
  EXPECT_FALSE(should_resolve_curve_pass_side(true, true, true));
}

TEST(V2XOvertakeCoreSpeed, HoldsActiveLineOnlyWithinBoundedTransientGapWindow)
{
  ActiveLineGapLossHoldRequest request;
  request.enabled = true;
  request.active_line = true;
  request.locked_target_seen = true;
  request.transient_gap_failure = true;
  request.now_sec = 10.4;
  request.last_valid_gap_sec = 10.0;
  request.hold_sec = 0.5;

  const auto active = resolve_active_line_gap_loss_hold(request);
  EXPECT_TRUE(active.active);
  EXPECT_NEAR(active.remaining_sec, 0.1, 1.0e-9);

  request.now_sec = 10.6;
  EXPECT_FALSE(resolve_active_line_gap_loss_hold(request).active);
}

TEST(V2XOvertakeCoreSpeed, GapHoldNeverMasksHardOrTargetFailures)
{
  ActiveLineGapLossHoldRequest request;
  request.enabled = true;
  request.active_line = true;
  request.locked_target_seen = true;
  request.transient_gap_failure = true;
  request.now_sec = 10.1;
  request.last_valid_gap_sec = 10.0;
  request.hold_sec = 0.5;

  request.locked_target_position_jump = true;
  EXPECT_FALSE(resolve_active_line_gap_loss_hold(request).active);
  request.locked_target_position_jump = false;
  request.explicit_forbidden_wp = true;
  EXPECT_FALSE(resolve_active_line_gap_loss_hold(request).active);
  request.explicit_forbidden_wp = false;
  request.emergency_brake = true;
  EXPECT_FALSE(resolve_active_line_gap_loss_hold(request).active);
  request.emergency_brake = false;
  request.transient_gap_failure = false;
  EXPECT_FALSE(resolve_active_line_gap_loss_hold(request).active);
}

TEST(V2XOvertakeCoreSpeed, ExplicitLineExclusivelyOwnsActiveOvertakeLateralPlan)
{
  OvertakeLateralPlannerOwnershipRequest request;
  request.explicit_line_enabled = true;
  request.behavior_requests_overtake = true;
  EXPECT_TRUE(explicit_overtake_line_owns_lateral_plan(request));

  request.behavior_requests_overtake = false;
  request.line_phase_active = true;
  EXPECT_TRUE(explicit_overtake_line_owns_lateral_plan(request));

  request.explicit_line_enabled = false;
  EXPECT_FALSE(explicit_overtake_line_owns_lateral_plan(request));
}

TEST(V2XOvertakeCoreSpeed, GapPlannerHardBoundsNeverOverrideExplicitLine)
{
  GapPlannerStateBoundsRequest request;
  EXPECT_TRUE(should_apply_gap_planner_state_bounds(request));

  request.explicit_line_owns_plan = true;
  EXPECT_FALSE(should_apply_gap_planner_state_bounds(request));
}

TEST(V2XOvertakeCoreSpeed, LiveCorridorLossDoesNotAbortLaterallyCommittedPass)
{
  LiveExecutionCorridorBlockRequest request;
  EXPECT_FALSE(should_block_live_execution_corridor(request));

  request.raw_corridor_blocked = true;
  EXPECT_TRUE(should_block_live_execution_corridor(request));

  request.pass_phase = true;
  EXPECT_TRUE(should_block_live_execution_corridor(request));

  request.lateral_clearance_latched = true;
  EXPECT_FALSE(should_block_live_execution_corridor(request));

  request.pass_phase = false;
  EXPECT_TRUE(should_block_live_execution_corridor(request));
}

TEST(V2XOvertakeCoreSpeed, DetectsLockedTargetCrossingSelectedPassSideOrdering)
{
  LockedTargetPassSideIntrusionRequest request;
  request.active_line = true;
  request.pass_side_sign = -1;
  request.target_seen = true;
  request.target_longitudinal_m = 2.0;
  request.ordering_margin_m = 0.10;

  // Successful right pass: target remains left of ego in common-course coordinates.
  request.target_relative_lateral_m = 0.67;
  EXPECT_FALSE(locked_target_intrudes_pass_side(request));

  // Failed right pass: target is on the selected/right side of ego.
  request.target_relative_lateral_m = -0.35;
  EXPECT_TRUE(locked_target_intrudes_pass_side(request));

  // Once Pass has proved lateral separation, a rotating hairpin projection
  // must not reinterpret the same committed maneuver as target intrusion.
  request.lateral_clearance_latched = true;
  EXPECT_FALSE(locked_target_intrudes_pass_side(request));
  request.lateral_clearance_latched = false;

  // A nearly coincident line is also an intrusion, before geometric crossing.
  request.target_relative_lateral_m = 0.05;
  EXPECT_TRUE(locked_target_intrudes_pass_side(request));

  request.target_longitudinal_m = -0.01;
  EXPECT_FALSE(locked_target_intrudes_pass_side(request));
  request.target_longitudinal_m = 2.0;
  request.target_position_jump = true;
  EXPECT_FALSE(locked_target_intrudes_pass_side(request));
}

TEST(V2XOvertakeCoreSpeed, RejectsNewSidePassAfterTargetIsAlreadyBehind)
{
  SideOvertakeEntryRequest request;
  request.target_longitudinal_m = -1.47;
  request.rear_tolerance_m = 0.5;
  EXPECT_FALSE(can_start_side_overtake(request));

  request.target_longitudinal_m = -0.4;
  EXPECT_TRUE(can_start_side_overtake(request));

  request.target_longitudinal_m = -1.47;
  request.continuing_overtake = true;
  EXPECT_TRUE(can_start_side_overtake(request));
}

TEST(V2XOvertakeCoreSpeed, RearSideTargetDoesNotForceFollowOnClearRoad)
{
  EXPECT_TRUE(side_only_target_requires_follow(0.2, 0.5));
  EXPECT_TRUE(side_only_target_requires_follow(-0.5, 0.5));
  EXPECT_FALSE(side_only_target_requires_follow(-0.51, 0.5));

  EXPECT_TRUE(side_only_target_requires_follow(
    std::numeric_limits<double>::quiet_NaN(), 0.5));
  EXPECT_TRUE(side_only_target_requires_follow(-1.0, -0.1));
}

TEST(V2XOvertakeCoreSpeed, SlowsClosingOnlyUntilPassLateralClearanceLatches)
{
  EXPECT_DOUBLE_EQ(resolve_unlatched_pass_closing_speed(1.5, 0.5, true, false), 0.5);
  EXPECT_DOUBLE_EQ(resolve_unlatched_pass_closing_speed(1.5, 0.5, false, false), 1.5);
  EXPECT_DOUBLE_EQ(resolve_unlatched_pass_closing_speed(1.5, 0.5, true, true), 1.5);
}

TEST(V2XOvertakeCoreSpeed, AdvancesExplicitLineRampWithTraveledPhaseDistance)
{
  OvertakeLineHorizonProgressRequest request;
  request.phase_distance_m = 8.0;
  request.horizon_distance_m = 2.0;

  EXPECT_NEAR(resolve_overtake_line_horizon_progress(request), 0.15625, 1e-9);

  request.phase_traveled_m = 4.0;
  EXPECT_NEAR(resolve_overtake_line_horizon_progress(request), 0.84375, 1e-9);

  request.phase_traveled_m = 6.0;
  EXPECT_DOUBLE_EQ(resolve_overtake_line_horizon_progress(request), 1.0);

  request.hold_target = true;
  request.phase_distance_m = 0.0;
  EXPECT_DOUBLE_EQ(resolve_overtake_line_horizon_progress(request), 1.0);
}

TEST(V2XOvertakeCoreSpeed, RejectsInvalidExplicitLineRampInputs)
{
  OvertakeLineHorizonProgressRequest request;
  request.phase_distance_m = 8.0;
  request.phase_traveled_m = -0.1;
  EXPECT_DOUBLE_EQ(resolve_overtake_line_horizon_progress(request), 0.0);

  request.phase_traveled_m = 0.0;
  request.horizon_distance_m = std::numeric_limits<double>::infinity();
  EXPECT_DOUBLE_EQ(resolve_overtake_line_horizon_progress(request), 0.0);
}

TEST(V2XOvertakeCoreSpeed, BuildsHeadingReferenceFromExplicitLateralLine)
{
  OvertakeLineHeadingReferenceRequest request;
  request.previous_lateral_m = 0.0;
  request.current_lateral_m = 0.2;
  request.delta_s_m = 1.0;
  EXPECT_NEAR(resolve_overtake_line_heading_reference(request), std::atan(0.2), 1e-12);

  request.previous_lateral_m = 0.2;
  EXPECT_DOUBLE_EQ(resolve_overtake_line_heading_reference(request), 0.0);

  request.previous_lateral_m = 0.0;
  request.base_curvature_radpm = 0.1;
  EXPECT_NEAR(
    resolve_overtake_line_heading_reference(request), std::atan2(0.2, 0.98), 1e-12);

  request.delta_s_m = 0.0;
  EXPECT_DOUBLE_EQ(resolve_overtake_line_heading_reference(request), 0.0);
}

TEST(V2XOvertakeCoreSpeed, PlacesPassGoalBeyondLockedTargetOnSelectedSide)
{
  PassSideLateralGoalRequest request;
  request.pass_side_sign = 1;
  request.base_lateral_offset_m = 1.2;
  request.target_lateral_m = 0.6;
  request.minimum_separation_m = 1.0;
  EXPECT_DOUBLE_EQ(resolve_pass_side_lateral_goal(request), 1.6);

  request.pass_side_sign = -1;
  request.target_lateral_m = -0.7;
  EXPECT_DOUBLE_EQ(resolve_pass_side_lateral_goal(request), -1.7);

  request.target_lateral_m = 0.0;
  EXPECT_DOUBLE_EQ(resolve_pass_side_lateral_goal(request), -1.2);
}

TEST(V2XOvertakeCoreSpeed, FallsBackToBasePassGoalWithoutTargetLateral)
{
  PassSideLateralGoalRequest request;
  request.pass_side_sign = 1;
  request.base_lateral_offset_m = 1.2;
  request.target_lateral_m = std::numeric_limits<double>::infinity();
  request.minimum_separation_m = 1.0;
  EXPECT_DOUBLE_EQ(resolve_pass_side_lateral_goal(request), 1.2);

  request.pass_side_sign = 0;
  EXPECT_DOUBLE_EQ(resolve_pass_side_lateral_goal(request), 0.0);
}

TEST(V2XOvertakeCoreSpeed, FixedBreakoutGoalDoesNotChaseTurningTarget)
{
  PassSideLateralGoalRequest request;
  request.pass_side_sign = -1;
  request.base_lateral_offset_m = 1.2;
  request.target_lateral_m = 0.0;
  request.minimum_separation_m = 0.75;
  request.fixed_lateral_goal_m = -1.2;
  EXPECT_DOUBLE_EQ(resolve_pass_side_lateral_goal(request), -1.2);

  request.target_lateral_m = -1.66;
  EXPECT_DOUBLE_EQ(resolve_pass_side_lateral_goal(request), -1.2);

  request.fixed_lateral_goal_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(resolve_pass_side_lateral_goal(request), -2.41);
}

TEST(V2XOvertakeCoreSpeed, ResolvesCenterOfValidatedPassCorridor)
{
  PassCorridorCenterRequest request;
  request.active = true;
  request.lower_bound_m = -2.1;
  request.upper_bound_m = -0.7;
  const auto center = resolve_pass_corridor_center(request);
  ASSERT_TRUE(center.has_value());
  EXPECT_DOUBLE_EQ(center.value(), -1.4);

  request.active = false;
  EXPECT_FALSE(resolve_pass_corridor_center(request).has_value());

  request.active = true;
  request.lower_bound_m = 0.8;
  request.upper_bound_m = 0.2;
  EXPECT_FALSE(resolve_pass_corridor_center(request).has_value());

  request.lower_bound_m = std::numeric_limits<double>::quiet_NaN();
  request.upper_bound_m = 1.0;
  EXPECT_FALSE(resolve_pass_corridor_center(request).has_value());
}

TEST(V2XOvertakeCoreSpeed, AdaptsShiftOutClosingSpeedToFrontDistanceBudget)
{
  AdaptiveShiftOutClosingSpeedRequest request;
  request.minimum_closing_speed_mps = 1.5;
  request.maximum_closing_speed_mps = 2.0;
  request.front_distance_m = 6.9;
  request.protected_front_distance_m = 5.0;
  request.remaining_shiftout_distance_m = 8.0;
  request.ego_speed_mps = 3.0;
  request.minimum_speed_mps = 1.0;
  request.minimum_time_sec = 0.5;

  auto result = resolve_adaptive_shiftout_closing_speed(request);
  EXPECT_DOUBLE_EQ(result.closing_speed_mps, 1.5);
  EXPECT_NEAR(result.remaining_time_sec, 8.0 / 3.0, 1e-9);
  EXPECT_NEAR(result.distance_budget_m, 1.9, 1e-9);

  request.front_distance_m = 8.5;
  request.ego_speed_mps = 4.0;
  result = resolve_adaptive_shiftout_closing_speed(request);
  EXPECT_DOUBLE_EQ(result.closing_speed_mps, 1.75);

  request.front_distance_m = 15.0;
  result = resolve_adaptive_shiftout_closing_speed(request);
  EXPECT_DOUBLE_EQ(result.closing_speed_mps, 2.0);
}

TEST(V2XOvertakeCoreSpeed, AdaptiveShiftOutCanMatchFrontSpeedAtSmallDistanceBudget)
{
  AdaptiveShiftOutClosingSpeedRequest request;
  request.minimum_closing_speed_mps = 0.0;
  request.maximum_closing_speed_mps = 1.5;
  request.front_distance_m = 7.0;
  request.protected_front_distance_m = 5.0;
  request.remaining_shiftout_distance_m = 8.0;
  request.ego_speed_mps = 3.0;
  request.minimum_speed_mps = 1.0;
  request.minimum_time_sec = 0.5;

  const auto result = resolve_adaptive_shiftout_closing_speed(request);
  EXPECT_NEAR(result.closing_speed_mps, 0.75, 1e-9);
}

TEST(V2XOvertakeCoreSpeed, RejectsInvalidAdaptiveShiftOutRange)
{
  AdaptiveShiftOutClosingSpeedRequest request{
    2.0, 1.5, 10.0, 5.0, 8.0, 4.0, 1.0, 0.5};
  EXPECT_THROW(resolve_adaptive_shiftout_closing_speed(request), std::invalid_argument);
}

TEST(V2XOvertakeCorePrediction, AccumulatesPathTimeAndSaturatesAtHorizon)
{
  double elapsed = advance_prediction_time(PredictionTimeRequest{0.0, 5.0, 10.0, 1.0, 1.0});
  EXPECT_DOUBLE_EQ(elapsed, 0.5);
  elapsed = advance_prediction_time(PredictionTimeRequest{elapsed, 2.0, 0.0, 2.0, 1.0});
  EXPECT_DOUBLE_EQ(elapsed, 1.0);

  auto invalid = PredictionTimeRequest{0.0, 1.0, 1.0, 0.0, 1.0};
  EXPECT_THROW(advance_prediction_time(invalid), std::invalid_argument);
}

TEST(V2XOvertakeCoreProgress, UsesAlongCourseDistanceAndProjectedSpeed)
{
  const std::vector<CoursePoint> path{{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};
  ForwardCourseProjectionRequest request;
  request.start_index = 0U;
  request.origin_x_m = 1.0;
  request.origin_y_m = 0.0;
  request.target_x_m = 15.0;
  request.target_y_m = 1.0;
  request.target_vx_mps = 5.0;
  request.lookbehind_distance_m = 2.0;
  request.lookahead_distance_m = 20.0;
  request.max_cross_track_distance_m = 2.0;

  const auto result = project_forward_course_progress(path, request);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.forward_distance_m, 14.0, 1e-12);
  EXPECT_NEAR(result.lateral_m, 1.0, 1e-12);
  EXPECT_NEAR(result.along_track_speed_mps, 5.0, 1e-12);
  EXPECT_EQ(result.segment_index, 1U);
}

TEST(V2XOvertakeCoreProgress, DetectsFrontAroundHairpinOutsideEgoTangent)
{
  const std::vector<CoursePoint> path{
    {0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}};
  ForwardCourseProjectionRequest request;
  request.start_index = 0U;
  request.origin_x_m = 1.0;
  request.origin_y_m = 0.0;
  request.target_x_m = 10.0;
  request.target_y_m = 5.0;
  request.target_vy_mps = 3.0;
  request.lookbehind_distance_m = 2.0;
  request.lookahead_distance_m = 20.0;
  request.max_cross_track_distance_m = 2.0;

  const auto result = project_forward_course_progress(path, request);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.forward_distance_m, 14.0, 1e-12);
  EXPECT_NEAR(result.lateral_m, 0.0, 1e-12);
  EXPECT_NEAR(result.along_track_speed_mps, 3.0, 1e-12);
}

TEST(V2XOvertakeCoreProgress, WrapsAcrossCircularCourseEnd)
{
  const std::vector<CoursePoint> path{
    {0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}};
  ForwardCourseProjectionRequest request;
  request.start_index = 3U;
  request.circular = true;
  request.origin_x_m = 0.0;
  request.origin_y_m = 9.0;
  request.target_x_m = 1.0;
  request.target_y_m = 0.0;
  request.target_vx_mps = 1.0;
  request.lookbehind_distance_m = 2.0;
  request.lookahead_distance_m = 15.0;
  request.max_cross_track_distance_m = 1.0;

  const auto result = project_forward_course_progress(path, request);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.forward_distance_m, 10.0, 1e-12);
  EXPECT_EQ(result.segment_index, 0U);
}

TEST(V2XOvertakeCoreProgress, WrapsAcrossLegacyDuplicateCircularEndpoint)
{
  const std::vector<CoursePoint> path{
    {0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}, {0.0, 0.0}};
  ForwardCourseProjectionRequest request;
  request.start_index = 3U;
  request.circular = true;
  request.origin_x_m = 0.0;
  request.origin_y_m = 1.0;
  request.target_x_m = 1.0;
  request.target_y_m = 0.0;
  request.target_vx_mps = 1.0;
  request.lookbehind_distance_m = 2.0;
  request.lookahead_distance_m = 5.0;
  request.max_cross_track_distance_m = 1.0;

  const auto result = project_forward_course_progress(path, request);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.forward_distance_m, 2.0, 1e-12);
  EXPECT_EQ(result.segment_index, 0U);
}

TEST(V2XOvertakeCoreProgress, SkipsRepeatedInteriorPoint)
{
  const std::vector<CoursePoint> path{
    {0.0, 0.0}, {10.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};
  ForwardCourseProjectionRequest request;
  request.start_index = 0U;
  request.origin_x_m = 1.0;
  request.target_x_m = 15.0;
  request.target_vx_mps = 1.0;
  request.lookbehind_distance_m = 2.0;
  request.lookahead_distance_m = 20.0;
  request.max_cross_track_distance_m = 1.0;

  const auto result = project_forward_course_progress(path, request);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.forward_distance_m, 14.0, 1e-12);
  EXPECT_EQ(result.segment_index, 2U);
}

TEST(V2XOvertakeCoreProgress, KeepsRearNegativeAndRejectsOppositeDirection)
{
  const std::vector<CoursePoint> path{{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};
  ForwardCourseProjectionRequest request;
  request.start_index = 1U;
  request.origin_x_m = 11.0;
  request.target_x_m = 9.0;
  request.target_vx_mps = 1.0;
  request.lookbehind_distance_m = 5.0;
  request.lookahead_distance_m = 8.0;
  request.max_cross_track_distance_m = 1.0;

  auto result = project_forward_course_progress(path, request);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.forward_distance_m, -2.0, 1e-12);

  request.start_index = 0U;
  request.origin_x_m = 1.0;
  request.target_x_m = 5.0;
  request.target_vx_mps = -1.0;
  result = project_forward_course_progress(path, request);
  EXPECT_FALSE(result.valid);
}

TEST(V2XOvertakeCoreProgress, RejectsTargetOutsideBoundedCourseSection)
{
  const std::vector<CoursePoint> path{{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};
  ForwardCourseProjectionRequest request;
  request.start_index = 0U;
  request.origin_x_m = 1.0;
  request.target_x_m = 30.0;
  request.target_vx_mps = 1.0;
  request.lookbehind_distance_m = 2.0;
  request.lookahead_distance_m = 10.0;
  request.max_cross_track_distance_m = 2.0;

  EXPECT_FALSE(project_forward_course_progress(path, request).valid);
}

TEST(V2XOvertakeCoreCompletion, RequiresRearClearBeforeHardCurve)
{
  PassCompletionRequest request;
  request.distance_to_hard_curve_m = 45.0;
  request.curve_buffer_m = 2.0;
  request.front_distance_m = 4.0;
  request.front_speed_mps = 8.0;
  request.planned_ego_speed_mps = 11.0;
  request.return_clear_distance_m = 4.0;
  request.minimum_shift_distance_m = 8.0;
  request.merge_buffer_m = 3.0;
  request.minimum_relative_speed_mps = 0.5;

  auto result = resolve_pass_completion(request);
  EXPECT_TRUE(result.feasible);
  EXPECT_NEAR(result.available_distance_m, 43.0, 1e-12);
  EXPECT_NEAR(result.required_distance_m, 11.0 * 8.0 / 3.0 + 3.0, 1e-12);

  request.distance_to_hard_curve_m = 30.0;
  result = resolve_pass_completion(request);
  EXPECT_FALSE(result.feasible);

  request.distance_to_hard_curve_m = std::numeric_limits<double>::infinity();
  result = resolve_pass_completion(request);
  EXPECT_TRUE(result.feasible);
}

TEST(V2XOvertakeCoreCompletion, RejectsInsufficientRelativeSpeed)
{
  PassCompletionRequest request;
  request.distance_to_hard_curve_m = 100.0;
  request.front_distance_m = 3.0;
  request.front_speed_mps = 10.8;
  request.planned_ego_speed_mps = 11.0;
  request.return_clear_distance_m = 4.0;
  request.minimum_shift_distance_m = 8.0;
  request.merge_buffer_m = 3.0;
  request.minimum_relative_speed_mps = 0.5;

  const auto result = resolve_pass_completion(request);
  EXPECT_FALSE(result.feasible);
  EXPECT_TRUE(std::isinf(result.required_distance_m));
}

TEST(V2XOvertakeCoreGuardPhase, KeepsEntryDistanceAndPrepareCheckBeforePassStarts)
{
  const auto result = resolve_overtake_guard_phase(
    OvertakeGuardPhaseRequest{false, 5.0, 2.5});

  EXPECT_DOUBLE_EQ(result.min_front_distance_m, 5.0);
  EXPECT_TRUE(result.require_prepare_distance);
}

TEST(V2XOvertakeCoreGuardPhase, UsesContinuationDistanceWithoutEntryPrepareCheck)
{
  const auto result = resolve_overtake_guard_phase(
    OvertakeGuardPhaseRequest{true, 5.0, 2.5});

  EXPECT_DOUBLE_EQ(result.min_front_distance_m, 2.5);
  EXPECT_FALSE(result.require_prepare_distance);
}

TEST(V2XOvertakeCoreGuardPhase, RejectsContinuationThresholdAboveEntryThreshold)
{
  EXPECT_THROW(
    resolve_overtake_guard_phase(OvertakeGuardPhaseRequest{true, 2.5, 5.0}),
    std::invalid_argument);
}

TEST(V2XOvertakeCoreCurveContinuation, AllowsOuterActivePassInSoftCurve)
{
  OvertakeCurveContinuationRequest request;
  request.continuation_enabled = true;
  request.continuing_overtake = true;
  request.soft_curve_forbidden = true;

  EXPECT_TRUE(can_continue_overtake_in_soft_curve(request));
}

TEST(V2XOvertakeCoreCurveContinuation, InnerPassRequiresExplicitExperimentFlag)
{
  OvertakeCurveContinuationRequest request;
  request.continuation_enabled = true;
  request.continuing_overtake = true;
  request.soft_curve_forbidden = true;
  request.inner_curve_pass = true;

  EXPECT_FALSE(can_continue_overtake_in_soft_curve(request));
  request.inner_soft_curve_enabled = true;
  EXPECT_TRUE(can_continue_overtake_in_soft_curve(request));
}

TEST(V2XOvertakeCoreCurveContinuation, HardCurveAndEmergencyAlwaysBlock)
{
  OvertakeCurveContinuationRequest request;
  request.continuation_enabled = true;
  request.inner_soft_curve_enabled = true;
  request.continuing_overtake = true;
  request.soft_curve_forbidden = true;
  request.inner_curve_pass = true;

  request.hard_curve_forbidden = true;
  EXPECT_FALSE(can_continue_overtake_in_soft_curve(request));
  request.hard_curve_forbidden = false;
  request.emergency_brake = true;
  EXPECT_FALSE(can_continue_overtake_in_soft_curve(request));
}

TEST(V2XOvertakeCoreCurveContinuation, NeverRelaxesNewPassOrCooldown)
{
  OvertakeCurveContinuationRequest request;
  request.continuation_enabled = true;
  request.inner_soft_curve_enabled = true;
  request.soft_curve_forbidden = true;

  EXPECT_FALSE(can_continue_overtake_in_soft_curve(request));
  request.continuing_overtake = true;
  request.cooldown_active = true;
  EXPECT_FALSE(can_continue_overtake_in_soft_curve(request));
}

OuterCurveOvertakeRequest outer_curve_request()
{
  OuterCurveOvertakeRequest request;
  request.entry_enabled = true;
  request.hard_continuation_enabled = true;
  request.soft_curve_forbidden = true;
  request.gap_available = true;
  request.locked_target_seen = true;
  request.pass_side_sign = -1;
  request.inner_curve_pass_side = 1;
  return request;
}

TEST(V2XOvertakeCoreOuterCurve, AllowsNewOuterEntryInSoftCurve)
{
  const auto resolution = resolve_outer_curve_overtake(outer_curve_request());

  EXPECT_TRUE(resolution.entry_allowed);
  EXPECT_FALSE(resolution.hard_continuation_allowed);
}

TEST(V2XOvertakeCoreOuterCurve, RejectsInnerEntryAndHardCurveEntry)
{
  auto request = outer_curve_request();
  request.pass_side_sign = request.inner_curve_pass_side;
  EXPECT_FALSE(resolve_outer_curve_overtake(request).entry_allowed);

  request = outer_curve_request();
  request.hard_curve_forbidden = true;
  EXPECT_FALSE(resolve_outer_curve_overtake(request).entry_allowed);
}

TEST(V2XOvertakeCoreOuterCurve, AllowsConfiguredNewOuterEntryInHardCurve)
{
  auto request = outer_curve_request();
  request.hard_entry_enabled = true;
  request.hard_curve_forbidden = true;

  const auto resolution = resolve_outer_curve_overtake(request);
  EXPECT_FALSE(resolution.entry_allowed);
  EXPECT_TRUE(resolution.hard_entry_allowed);
  EXPECT_FALSE(resolution.hard_continuation_allowed);
}

TEST(V2XOvertakeCoreOuterCurve, ContinuesLockedOuterLineThroughHardCurve)
{
  auto request = outer_curve_request();
  request.continuing_overtake = true;
  request.hard_curve_forbidden = true;

  const auto resolution = resolve_outer_curve_overtake(request);

  EXPECT_FALSE(resolution.entry_allowed);
  EXPECT_TRUE(resolution.hard_continuation_allowed);
}

TEST(V2XOvertakeCoreOuterCurve, HardContinuationRequiresGapAndLockedTarget)
{
  auto request = outer_curve_request();
  request.continuing_overtake = true;
  request.hard_curve_forbidden = true;
  request.gap_available = false;
  EXPECT_FALSE(resolve_outer_curve_overtake(request).hard_continuation_allowed);

  request.gap_available = true;
  request.locked_target_seen = false;
  EXPECT_FALSE(resolve_outer_curve_overtake(request).hard_continuation_allowed);
}

TEST(V2XOvertakeCoreOuterCurve, NeverRelaxesExplicitAndEmergencyGuards)
{
  auto request = outer_curve_request();
  request.explicit_forbidden_wp = true;
  EXPECT_FALSE(resolve_outer_curve_overtake(request).entry_allowed);

  request = outer_curve_request();
  request.cooldown_active = true;
  EXPECT_FALSE(resolve_outer_curve_overtake(request).entry_allowed);

  request = outer_curve_request();
  request.emergency_brake = true;
  EXPECT_FALSE(resolve_outer_curve_overtake(request).entry_allowed);

  request = outer_curve_request();
  request.hard_entry_enabled = true;
  request.hard_curve_forbidden = true;
  request.explicit_forbidden_wp = true;
  EXPECT_FALSE(resolve_outer_curve_overtake(request).hard_entry_allowed);
}

TEST(V2XOvertakeCoreOuterCurve, DisabledFlagsPreserveLegacyCurvePolicy)
{
  auto request = outer_curve_request();
  request.entry_enabled = false;
  EXPECT_FALSE(resolve_outer_curve_overtake(request).entry_allowed);

  request = outer_curve_request();
  request.continuing_overtake = true;
  request.hard_curve_forbidden = true;
  request.hard_continuation_enabled = false;
  EXPECT_FALSE(resolve_outer_curve_overtake(request).hard_continuation_allowed);
}

InnerCurveOvertakeRequest inner_curve_request()
{
  InnerCurveOvertakeRequest request;
  request.entry_enabled = true;
  request.hard_continuation_enabled = true;
  request.soft_curve_forbidden = true;
  request.gap_available = true;
  request.locked_target_seen = true;
  request.pass_side_sign = 1;
  request.inner_curve_pass_side = 1;
  return request;
}

TEST(V2XOvertakeCoreInnerCurve, AllowsNewInnerEntryInSoftCurve)
{
  const auto resolution = resolve_inner_curve_overtake(inner_curve_request());

  EXPECT_TRUE(resolution.entry_allowed);
  EXPECT_FALSE(resolution.hard_continuation_allowed);
}

TEST(V2XOvertakeCoreInnerCurve, RejectsOuterEntryAndHardCurveEntry)
{
  auto request = inner_curve_request();
  request.pass_side_sign = -request.inner_curve_pass_side;
  EXPECT_FALSE(resolve_inner_curve_overtake(request).entry_allowed);

  request = inner_curve_request();
  request.hard_curve_forbidden = true;
  EXPECT_FALSE(resolve_inner_curve_overtake(request).entry_allowed);
}

TEST(V2XOvertakeCoreInnerCurve, AllowsConfiguredNewInnerEntryInHardCurve)
{
  auto request = inner_curve_request();
  request.hard_entry_enabled = true;
  request.hard_curve_forbidden = true;

  const auto resolution = resolve_inner_curve_overtake(request);
  EXPECT_FALSE(resolution.entry_allowed);
  EXPECT_TRUE(resolution.hard_entry_allowed);
  EXPECT_FALSE(resolution.hard_continuation_allowed);
}

TEST(V2XOvertakeCoreInnerCurve, ContinuesLockedInnerLineThroughHardCurve)
{
  auto request = inner_curve_request();
  request.continuing_overtake = true;
  request.hard_curve_forbidden = true;

  const auto resolution = resolve_inner_curve_overtake(request);

  EXPECT_FALSE(resolution.entry_allowed);
  EXPECT_TRUE(resolution.hard_continuation_allowed);
}

TEST(V2XOvertakeCoreInnerCurve, HardContinuationRequiresGapAndLockedTarget)
{
  auto request = inner_curve_request();
  request.continuing_overtake = true;
  request.hard_curve_forbidden = true;
  request.gap_available = false;
  EXPECT_FALSE(resolve_inner_curve_overtake(request).hard_continuation_allowed);

  request.gap_available = true;
  request.locked_target_seen = false;
  EXPECT_FALSE(resolve_inner_curve_overtake(request).hard_continuation_allowed);
}

TEST(V2XOvertakeCoreInnerCurve, NeverRelaxesExplicitCooldownAndEmergencyGuards)
{
  auto request = inner_curve_request();
  request.explicit_forbidden_wp = true;
  EXPECT_FALSE(resolve_inner_curve_overtake(request).entry_allowed);

  request = inner_curve_request();
  request.cooldown_active = true;
  EXPECT_FALSE(resolve_inner_curve_overtake(request).entry_allowed);

  request = inner_curve_request();
  request.emergency_brake = true;
  EXPECT_FALSE(resolve_inner_curve_overtake(request).entry_allowed);

  request = inner_curve_request();
  request.hard_entry_enabled = true;
  request.hard_curve_forbidden = true;
  request.cooldown_active = true;
  EXPECT_FALSE(resolve_inner_curve_overtake(request).hard_entry_allowed);
}

TEST(V2XOvertakeCoreInnerCurve, DisabledFlagsPreserveLegacyCurvePolicy)
{
  auto request = inner_curve_request();
  request.entry_enabled = false;
  EXPECT_FALSE(resolve_inner_curve_overtake(request).entry_allowed);

  request = inner_curve_request();
  request.continuing_overtake = true;
  request.hard_curve_forbidden = true;
  request.hard_continuation_enabled = false;
  EXPECT_FALSE(resolve_inner_curve_overtake(request).hard_continuation_allowed);
}

ActiveHardCurveContinuationRequest active_hard_curve_request()
{
  ActiveHardCurveContinuationRequest request;
  request.enabled = true;
  request.continuing_overtake = true;
  request.pass_phase = true;
  request.locked_target_seen = true;
  request.hard_curve_ahead = true;
  request.completion = PassCompletionRequest{
    12.0, 0.5, 3.0, 3.0, 6.0, 0.5, 0.0, 0.0, 0.5};
  return request;
}

TEST(V2XOvertakeCoreActiveHardCurve, AllowsFeasibleLockedPassBeforeBoundary)
{
  const auto resolution =
    resolve_active_hard_curve_continuation(active_hard_curve_request());

  EXPECT_TRUE(resolution.allowed);
  EXPECT_NEAR(resolution.completion.available_distance_m, 11.5, 1e-9);
  EXPECT_NEAR(resolution.completion.required_distance_m, 7.0, 1e-9);
}

TEST(V2XOvertakeCoreActiveHardCurve, RejectsInsufficientBoundaryDistance)
{
  auto request = active_hard_curve_request();
  request.completion.distance_to_hard_curve_m = 4.0;

  const auto resolution = resolve_active_hard_curve_continuation(request);

  EXPECT_FALSE(resolution.allowed);
  EXPECT_NEAR(resolution.completion.available_distance_m, 3.5, 1e-9);
  EXPECT_NEAR(resolution.completion.required_distance_m, 7.0, 1e-9);
}

TEST(V2XOvertakeCoreActiveHardCurve, HoldsLaterallyLatchedPassAtBoundary)
{
  auto request = active_hard_curve_request();
  request.completion.distance_to_hard_curve_m = 4.0;
  request.lateral_clearance_latched = true;

  const auto resolution = resolve_active_hard_curve_continuation(request);

  EXPECT_TRUE(resolution.allowed);
  EXPECT_FALSE(resolution.completion.feasible);
}

TEST(V2XOvertakeCoreActiveHardCurve, NeverRelaxesEntryShiftoutOrSafetyGuards)
{
  auto request = active_hard_curve_request();

  request.continuing_overtake = false;
  EXPECT_FALSE(resolve_active_hard_curve_continuation(request).allowed);
  request.continuing_overtake = true;
  request.pass_phase = false;
  EXPECT_FALSE(resolve_active_hard_curve_continuation(request).allowed);
  request.pass_phase = true;
  request.locked_target_seen = false;
  EXPECT_FALSE(resolve_active_hard_curve_continuation(request).allowed);
  request.locked_target_seen = true;
  request.explicit_forbidden_wp = true;
  EXPECT_FALSE(resolve_active_hard_curve_continuation(request).allowed);
  request.explicit_forbidden_wp = false;
  request.cooldown_active = true;
  EXPECT_FALSE(resolve_active_hard_curve_continuation(request).allowed);
  request.cooldown_active = false;
  request.emergency_brake = true;
  EXPECT_FALSE(resolve_active_hard_curve_continuation(request).allowed);
}

TEST(V2XOvertakeCoreSide, SelectsPreferredWhenBothSidesAreFeasible)
{
  const auto result = select_pass_side(
    SideSelectionRequest{PassSide::Left, PassSide::None, true, true, true});
  EXPECT_EQ(result.side, PassSide::Left);
  EXPECT_EQ(result.reason, SideSelectionReason::Preferred);
}

TEST(V2XOvertakeCoreSide, SelectsAlternateWhenPreferredSideIsBlocked)
{
  const auto result = select_pass_side(
    SideSelectionRequest{PassSide::Left, PassSide::None, false, true, true});
  EXPECT_EQ(result.side, PassSide::Right);
  EXPECT_EQ(result.reason, SideSelectionReason::Alternate);
}

TEST(V2XOvertakeCoreSide, DoesNotUseAlternateWhenItIsDisabled)
{
  const auto result = select_pass_side(
    SideSelectionRequest{PassSide::Left, PassSide::None, false, true, false});
  EXPECT_EQ(result.side, PassSide::None);
  EXPECT_EQ(result.reason, SideSelectionReason::PreferredUnavailable);
}

TEST(V2XOvertakeCoreSide, PreservesLockedSideEvenWhenPreferenceChanges)
{
  const auto result = select_pass_side(
    SideSelectionRequest{PassSide::Left, PassSide::Right, true, true, true});
  EXPECT_EQ(result.side, PassSide::Right);
  EXPECT_EQ(result.reason, SideSelectionReason::Locked);
}

TEST(V2XOvertakeCoreSide, NeverSwitchesAwayFromAnUnavailableLockedSide)
{
  const auto result = select_pass_side(
    SideSelectionRequest{PassSide::Left, PassSide::Right, true, false, true});
  EXPECT_EQ(result.side, PassSide::None);
  EXPECT_EQ(result.reason, SideSelectionReason::LockedUnavailable);
}

TEST(V2XOvertakeCoreSide, RejectsSelectionWithoutAValidPreference)
{
  const auto result = select_pass_side(
    SideSelectionRequest{PassSide::None, PassSide::None, true, true, true});
  EXPECT_EQ(result.side, PassSide::None);
  EXPECT_EQ(result.reason, SideSelectionReason::InvalidPreference);
}

TEST(V2XOvertakeCoreSide, LowSpeedPassPrefersReachableSideOverSlightlyWiderSide)
{
  const auto side = select_reachable_low_speed_pass_side(
    LowSpeedPassSideRequest{
      2.03,
      LowSpeedPassSideCandidate{true, 2.60, 2.40},
      LowSpeedPassSideCandidate{true, -2.71, 2.59}});
  EXPECT_EQ(side, PassSide::Left);
}

LowSpeedBypassCandidateRequest low_speed_bypass_request()
{
  LowSpeedBypassCandidateRequest request;
  request.enabled = true;
  request.candidate_vehicle_present = true;
  request.vehicle_speed_mps = 0.0;
  request.maximum_vehicle_speed_mps = 0.2;
  request.forward_distance_m = 3.2;
  request.minimum_prepare_distance_m = 3.0;
  request.maximum_entry_distance_m = 10.0;
  return request;
}

TEST(V2XOvertakeCoreLowSpeedBypass, AcceptsCloseStoppedVehicleAtPrepareBoundary)
{
  auto request = low_speed_bypass_request();
  EXPECT_TRUE(can_start_low_speed_bypass(request));

  request.forward_distance_m = request.minimum_prepare_distance_m;
  EXPECT_TRUE(can_start_low_speed_bypass(request));
  request.forward_distance_m = request.maximum_entry_distance_m;
  EXPECT_TRUE(can_start_low_speed_bypass(request));
}

TEST(V2XOvertakeCoreLowSpeedBypass, RejectsBumperTouchAndMovingVehicle)
{
  auto request = low_speed_bypass_request();
  request.forward_distance_m = 2.99;
  EXPECT_FALSE(can_start_low_speed_bypass(request));

  request = low_speed_bypass_request();
  request.vehicle_speed_mps = 0.21;
  EXPECT_FALSE(can_start_low_speed_bypass(request));
}

TEST(V2XOvertakeCoreLowSpeedBypass, StartGridSuppressionBlocksNewCandidate)
{
  auto request = low_speed_bypass_request();
  request.start_grid_stop_suppressed = true;
  EXPECT_FALSE(can_start_low_speed_bypass(request));

  request.start_grid_stop_suppressed = false;
  EXPECT_TRUE(can_start_low_speed_bypass(request));
}

TEST(V2XOvertakeCoreLowSpeedBypass, RaceSessionGateOnlyAppliesWithAwsimStateTracking)
{
  EXPECT_TRUE(is_v2x_behavior_session_active(false, false));
  EXPECT_FALSE(is_v2x_behavior_session_active(true, false));
  EXPECT_TRUE(is_v2x_behavior_session_active(true, true));
}

TEST(V2XOvertakeCoreLowSpeedBypass, AllowsSoftCurveOverrideButNotExplicitForbiddenWaypoint)
{
  auto request = low_speed_bypass_request();
  request.overtake_forbidden = true;
  request.ignore_soft_curve_forbidden = true;
  EXPECT_TRUE(can_start_low_speed_bypass(request));

  request.explicit_forbidden_wp = true;
  EXPECT_FALSE(can_start_low_speed_bypass(request));
  request.continuing = true;
  EXPECT_TRUE(can_start_low_speed_bypass(request));
}

TEST(V2XOvertakeCoreLowSpeedBypass, CandidateOwnsLineWhenLowSpeedBypassIsActive)
{
  StoppedVehicleLineOwnershipRequest request;
  request.low_speed_candidate = true;
  request.overtake_behavior_active = true;

  EXPECT_TRUE(should_yield_overtake_line_to_stopped_bypass(request));
}

TEST(V2XOvertakeCoreLowSpeedBypass, SelectedGenericOvertakeKeepsLineForCloseStoppedFront)
{
  StoppedVehicleLineOwnershipRequest request;
  request.overtake_behavior_active = true;
  request.has_front_vehicle = true;
  request.front_distance_m = 2.5;
  request.front_speed_mps = 0.0;
  request.maximum_stopped_speed_mps = 0.2;
  request.stopped_detection_distance_m = 18.0;

  EXPECT_FALSE(should_yield_overtake_line_to_stopped_bypass(request));
}

TEST(V2XOvertakeCoreLowSpeedBypass, FollowWithoutBypassReleasesStaleOvertakeLine)
{
  StoppedVehicleLineOwnershipRequest request;
  request.has_front_vehicle = true;
  request.front_distance_m = 4.0;
  request.front_speed_mps = 0.0;
  request.maximum_stopped_speed_mps = 0.2;
  request.stopped_detection_distance_m = 18.0;

  EXPECT_TRUE(should_yield_overtake_line_to_stopped_bypass(request));
}

TEST(V2XOvertakeCoreSide, LowSpeedPassUsesWidthOnlyForEqualTransitions)
{
  const auto side = select_reachable_low_speed_pass_side(
    LowSpeedPassSideRequest{
      0.0,
      LowSpeedPassSideCandidate{true, 2.0, 2.20},
      LowSpeedPassSideCandidate{true, -2.0, 2.60}});
  EXPECT_EQ(side, PassSide::Right);
}

TEST(V2XOvertakeCoreSide, LowSpeedPassRejectsInvalidFeasibleCandidate)
{
  const auto side = select_reachable_low_speed_pass_side(
    LowSpeedPassSideRequest{
      0.0,
      LowSpeedPassSideCandidate{true, std::numeric_limits<double>::quiet_NaN(), 2.20},
      LowSpeedPassSideCandidate{false, -2.0, 2.60}});
  EXPECT_EQ(side, PassSide::None);
}

TEST(V2XOvertakeCoreSide, LowSpeedPassHardensCorridorOnlyAfterEgoEnters)
{
  EXPECT_FALSE(has_entered_low_speed_pass_corridor(2.03, -4.0, -1.41));
  EXPECT_TRUE(has_entered_low_speed_pass_corridor(-1.38, -4.0, -1.41));
  EXPECT_TRUE(has_entered_low_speed_pass_corridor(-2.71, -4.0, -1.41));
}

TEST(V2XOvertakeCoreSide, LowSpeedPassRejectsInvalidCorridor)
{
  EXPECT_FALSE(has_entered_low_speed_pass_corridor(0.0, 1.0, -1.0));
  EXPECT_FALSE(has_entered_low_speed_pass_corridor(
      std::numeric_limits<double>::quiet_NaN(), -1.0, 1.0));
}

TEST(V2XOvertakeCoreSide, LowSpeedPassLimitsOnlyShiftPhaseVelocity)
{
  EXPECT_DOUBLE_EQ(resolve_low_speed_pass_velocity(11.1, 1.0, false), 1.0);
  EXPECT_DOUBLE_EQ(resolve_low_speed_pass_velocity(11.1, 1.0, true), 11.1);
  EXPECT_DOUBLE_EQ(resolve_low_speed_pass_velocity(0.8, 1.0, false), 0.8);
  EXPECT_THROW(resolve_low_speed_pass_velocity(-1.0, 1.0, false), std::invalid_argument);
}

TEST(V2XOvertakeCoreSide, LowSpeedShiftSteersTowardNegativeLateralTarget)
{
  LowSpeedShiftSteeringRequest request;
  request.current_lateral_m = 2.03;
  request.target_lateral_m = -2.71;
  request.current_heading_error_rad = 0.0;
  request.reference_curvature_radpm = -0.05;
  request.wheelbase_m = 1.087;
  request.max_steering_rad = 0.559;
  request.lateral_gain = 0.4;
  request.heading_gain = 1.3;
  EXPECT_DOUBLE_EQ(resolve_low_speed_shift_steering(request), -0.559);
}

TEST(V2XOvertakeCoreSide, SelectsPhaseSpecificDirectControlVelocity)
{
  EXPECT_DOUBLE_EQ(resolve_low_speed_direct_control_velocity(
      LowSpeedDirectControlPhase::Shift, 3.0, 6.0, 4.0, 11.1), 3.0);
  EXPECT_DOUBLE_EQ(resolve_low_speed_direct_control_velocity(
      LowSpeedDirectControlPhase::Pass, 3.0, 6.0, 4.0, 11.1), 6.0);
  EXPECT_DOUBLE_EQ(resolve_low_speed_direct_control_velocity(
      LowSpeedDirectControlPhase::Rejoin, 3.0, 6.0, 4.0, 11.1), 4.0);
  EXPECT_DOUBLE_EQ(resolve_low_speed_direct_control_velocity(
      LowSpeedDirectControlPhase::Pass, 3.0, 12.0, 4.0, 10.0), 10.0);
  EXPECT_THROW(resolve_low_speed_direct_control_velocity(
      LowSpeedDirectControlPhase::Pass, 3.0, -1.0, 4.0, 11.1), std::invalid_argument);
}

TEST(V2XOvertakeCoreSide, LowSpeedShiftHeadingFeedbackCountersteers)
{
  LowSpeedShiftSteeringRequest request;
  request.current_lateral_m = -2.0;
  request.target_lateral_m = -2.71;
  request.current_heading_error_rad = -0.5;
  request.reference_curvature_radpm = 0.0;
  request.wheelbase_m = 1.087;
  request.max_steering_rad = 0.559;
  request.lateral_gain = 0.4;
  request.heading_gain = 1.3;
  EXPECT_GT(resolve_low_speed_shift_steering(request), 0.0);
}

TEST(V2XOvertakeCoreSide, LowSpeedShiftRejectsInvalidGeometry)
{
  LowSpeedShiftSteeringRequest request;
  request.wheelbase_m = 0.0;
  request.max_steering_rad = 0.559;
  request.lateral_gain = 0.4;
  request.heading_gain = 1.3;
  EXPECT_THROW(resolve_low_speed_shift_steering(request), std::invalid_argument);
}

TEST(V2XOvertakeCoreSide, LowSpeedShiftCompletesOnlyAfterLateralAndHeadingSettle)
{
  EXPECT_FALSE(is_low_speed_shift_complete(-1.36, -0.75, -2.71, 0.4, 0.2));
  EXPECT_FALSE(is_low_speed_shift_complete(-2.60, -0.25, -2.71, 0.4, 0.2));
  EXPECT_TRUE(is_low_speed_shift_complete(-2.60, -0.15, -2.71, 0.4, 0.2));
  EXPECT_FALSE(is_low_speed_shift_complete(
      std::numeric_limits<double>::quiet_NaN(), 0.0, -2.71, 0.4, 0.2));
}

TEST(V2XOvertakeCoreSide, LowSpeedShiftBeginsRejoinOnlyAfterStoppedVehiclePackClears)
{
  EXPECT_FALSE(should_begin_low_speed_shift_rejoin(true, false, false, 2.0, 2.0));
  EXPECT_FALSE(should_begin_low_speed_shift_rejoin(false, true, false, 2.0, 2.0));
  EXPECT_FALSE(should_begin_low_speed_shift_rejoin(false, false, true, 2.0, 2.0));
  EXPECT_FALSE(should_begin_low_speed_shift_rejoin(false, false, false, 1.99, 2.0));
  EXPECT_TRUE(should_begin_low_speed_shift_rejoin(false, false, false, 2.0, 2.0));
  EXPECT_FALSE(should_begin_low_speed_shift_rejoin(
      false, false, false, std::numeric_limits<double>::quiet_NaN(), 2.0));
}

TEST(V2XOvertakeCoreSide, LowSpeedShiftReleasesOnlyAfterRejoinPoseSettles)
{
  EXPECT_FALSE(should_release_low_speed_shift_control(false, false, false, false, 2.0, 2.0));
  EXPECT_FALSE(should_release_low_speed_shift_control(true, true, false, false, 2.0, 2.0));
  EXPECT_FALSE(should_release_low_speed_shift_control(true, false, true, false, 2.0, 2.0));
  EXPECT_FALSE(should_release_low_speed_shift_control(true, false, false, true, 2.0, 2.0));
  EXPECT_FALSE(should_release_low_speed_shift_control(true, false, false, false, 1.99, 2.0));
  EXPECT_TRUE(should_release_low_speed_shift_control(true, false, false, false, 2.0, 2.0));
  EXPECT_FALSE(should_release_low_speed_shift_control(
      true, false, false, false, std::numeric_limits<double>::quiet_NaN(), 2.0));
}

TEST(V2XOvertakeCoreContinuity, HoldsShortTargetLossInsteadOfReturning)
{
  ContinuityRequest request;
  request.target_age_sec = 0.10;
  request.target_hold_sec = 0.30;
  EXPECT_EQ(resolve_target_continuity(request), ContinuityAction::Hold);

  request.target_age_sec = 0.31;
  EXPECT_EQ(resolve_target_continuity(request), ContinuityAction::Recovery);
}

TEST(V2XOvertakeCoreContinuity, HoldsLatchedActiveTargetAcrossGapRecheckLoss)
{
  ContinuityRequest request;
  request.target_seen = true;
  request.target_age_sec = 0.0;
  request.target_hold_sec = 0.30;
  request.active_execution_latched = true;
  EXPECT_EQ(resolve_target_continuity(request), ContinuityAction::Hold);

  request.active_execution_latched = false;
  EXPECT_EQ(resolve_target_continuity(request), ContinuityAction::Recovery);
}

TEST(V2XOvertakeCoreContinuity, ReturnsOnlyAfterRearClearConfirmation)
{
  ContinuityRequest request;
  request.target_seen = true;
  request.rear_clear_observed = true;
  request.target_age_sec = 0.0;
  request.target_hold_sec = 0.30;
  EXPECT_EQ(resolve_target_continuity(request), ContinuityAction::Hold);

  request.rear_clear_confirmed = true;
  EXPECT_EQ(resolve_target_continuity(request), ContinuityAction::Return);

  request.side_vehicle_present = true;
  EXPECT_EQ(resolve_target_continuity(request), ContinuityAction::Hold);

  request.rear_clear_observed = false;
  request.rear_clear_confirmed = false;
  request.target_not_ahead = true;
  EXPECT_EQ(resolve_target_continuity(request), ContinuityAction::Hold);
}

TEST(V2XOvertakeCoreContinuity, SolverFailureAndPositionJumpRequestRecovery)
{
  ContinuityRequest request;
  request.target_age_sec = 0.0;
  request.target_hold_sec = 0.30;
  request.solver_recovery_requested = true;
  EXPECT_EQ(resolve_target_continuity(request), ContinuityAction::Recovery);

  request.solver_recovery_requested = false;
  request.target_position_jump = true;
  EXPECT_EQ(resolve_target_continuity(request), ContinuityAction::Recovery);
}

TEST(V2XOvertakeCoreContinuity, ReacquiresOnlySameTargetAndSideDuringEarlyReturn)
{
  ReacquireRequest request;
  request.enabled = true;
  request.stable_target_id = true;
  request.same_target = true;
  request.same_side = true;
  request.gap_available = true;
  request.execution_allowed = true;
  request.return_elapsed_sec = 0.20;
  request.reacquire_window_sec = 0.50;
  request.return_progress = 0.10;
  request.max_return_progress = 0.25;
  EXPECT_TRUE(can_reacquire_during_return(request));

  request.same_target = false;
  EXPECT_FALSE(can_reacquire_during_return(request));
  request.same_target = true;
  request.same_side = false;
  EXPECT_FALSE(can_reacquire_during_return(request));
  request.same_side = true;
  request.return_elapsed_sec = 0.51;
  EXPECT_FALSE(can_reacquire_during_return(request));
  request.return_elapsed_sec = 0.20;
  request.return_progress = 0.26;
  EXPECT_FALSE(can_reacquire_during_return(request));
}

TEST(V2XOvertakeCoreRecovery, IntegratesEachAcceptedSpeedObservation)
{
  double distance = 0.0;
  auto resolution = integrate_forward_distance(
    ForwardDistanceRequest{distance, 3.0, 0.1, 0.2});
  ASSERT_TRUE(resolution.observation_accepted);
  distance = resolution.accumulated_distance_m;
  EXPECT_NEAR(distance, 0.3, 1e-12);

  resolution = integrate_forward_distance(
    ForwardDistanceRequest{distance, 0.0, 0.1, 0.2});
  ASSERT_TRUE(resolution.observation_accepted);
  distance = resolution.accumulated_distance_m;
  EXPECT_NEAR(distance, 0.3, 1e-12);

  resolution = integrate_forward_distance(
    ForwardDistanceRequest{distance, 1.0, 0.1, 0.2});
  ASSERT_TRUE(resolution.observation_accepted);
  EXPECT_NEAR(resolution.accumulated_distance_m, 0.4, 1e-12);
  EXPECT_NE(resolution.accumulated_distance_m, 1.0 * 0.3);
}

TEST(V2XOvertakeCoreRecovery, RejectsGapRollbackAndInvalidObservationWithoutDistanceJump)
{
  const ForwardDistanceRequest base{1.25, 2.0, 0.1, 0.2};

  auto request = base;
  request.delta_sec = 0.201;
  auto resolution = integrate_forward_distance(request);
  EXPECT_FALSE(resolution.observation_accepted);
  EXPECT_DOUBLE_EQ(resolution.accumulated_distance_m, 1.25);

  request = base;
  request.delta_sec = -0.001;
  resolution = integrate_forward_distance(request);
  EXPECT_FALSE(resolution.observation_accepted);
  EXPECT_DOUBLE_EQ(resolution.accumulated_distance_m, 1.25);

  request = base;
  request.forward_speed_mps = std::numeric_limits<double>::quiet_NaN();
  resolution = integrate_forward_distance(request);
  EXPECT_FALSE(resolution.observation_accepted);
  EXPECT_DOUBLE_EQ(resolution.accumulated_distance_m, 1.25);
}

TEST(V2XOvertakeCoreRecovery, KeepsConfiguredVelocityCeilingAtZeroVehicleSpeed)
{
  const auto resolution = resolve_recovery_policy(recovery_request());
  EXPECT_DOUBLE_EQ(resolution.velocity_limit_mps, 3.0);
  EXPECT_EQ(resolution.exit_reason, RecoveryExitReason::Active);
}

TEST(V2XOvertakeCoreRecovery, ResolvesEveryBoundedExitReason)
{
  auto request = recovery_request();
  request.lateral_error_m = 0.2;
  EXPECT_EQ(
    resolve_recovery_policy(request).exit_reason,
    RecoveryExitReason::LateralComplete);

  request = recovery_request();
  request.traveled_distance_m = 10.0;
  EXPECT_EQ(
    resolve_recovery_policy(request).exit_reason,
    RecoveryExitReason::DistanceComplete);

  request = recovery_request();
  request.stalled_sec = 1.0;
  EXPECT_EQ(resolve_recovery_policy(request).exit_reason, RecoveryExitReason::Stalled);

  request = recovery_request();
  request.elapsed_sec = 5.0;
  EXPECT_EQ(resolve_recovery_policy(request).exit_reason, RecoveryExitReason::TimedOut);

  request = recovery_request();
  request.elapsed_sec = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    resolve_recovery_policy(request).exit_reason,
    RecoveryExitReason::InvalidObservation);
}

TEST(V2XOvertakeCoreRecovery, RejectsInvalidPolicyConfiguration)
{
  auto request = recovery_request();
  request.configured_velocity_limit_mps = 0.0;
  EXPECT_THROW(resolve_recovery_policy(request), std::invalid_argument);

  request = recovery_request();
  request.stall_timeout_sec = 5.1;
  EXPECT_THROW(resolve_recovery_policy(request), std::invalid_argument);

  ForwardDistanceRequest distance_request{0.0, 1.0, 0.1, 0.0};
  EXPECT_THROW(integrate_forward_distance(distance_request), std::invalid_argument);
}

TEST(V2XOvertakeCoreStallWatchdog, TimesOutOnlyAcrossContinuousLowSpeedObservations)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  StallWatchdogRequest request{true, 0.0, 10.0, nan, nan, 0.15, 1.5, 0.2};

  auto resolution = update_stall_watchdog(request);
  EXPECT_TRUE(resolution.observation_accepted);
  EXPECT_FALSE(resolution.timed_out);
  EXPECT_DOUBLE_EQ(resolution.stall_since_sec, 10.0);

  request.previous_update_sec = resolution.update_sec;
  request.stall_since_sec = resolution.stall_since_sec;
  request.now_sec = 11.5;
  request.max_observation_gap_sec = 2.0;
  resolution = update_stall_watchdog(request);
  EXPECT_TRUE(resolution.timed_out);
  EXPECT_DOUBLE_EQ(resolution.stalled_sec, 1.5);
}

TEST(V2XOvertakeCoreStallWatchdog, ResetsForMotionInactiveStateAndObservationGap)
{
  StallWatchdogRequest request{true, 0.0, 10.1, 10.0, 9.0, 0.15, 1.5, 0.2};

  auto resolution = update_stall_watchdog(request);
  EXPECT_DOUBLE_EQ(resolution.stall_since_sec, 9.0);

  request.speed_mps = 0.16;
  resolution = update_stall_watchdog(request);
  EXPECT_FALSE(std::isfinite(resolution.stall_since_sec));
  EXPECT_FALSE(resolution.timed_out);

  request.speed_mps = 0.0;
  request.active = false;
  resolution = update_stall_watchdog(request);
  EXPECT_FALSE(std::isfinite(resolution.stall_since_sec));

  request.active = true;
  request.now_sec = 10.5;
  request.previous_update_sec = 10.1;
  request.stall_since_sec = 9.0;
  resolution = update_stall_watchdog(request);
  EXPECT_DOUBLE_EQ(resolution.stall_since_sec, 10.5);
  EXPECT_DOUBLE_EQ(resolution.stalled_sec, 0.0);
}

TEST(V2XOvertakeCoreStallWatchdog, RejectsInvalidConfigurationAndObservation)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  StallWatchdogRequest request{true, 0.0, 10.0, nan, nan, 0.15, 1.5, 0.2};

  request.timeout_sec = 0.0;
  EXPECT_THROW(update_stall_watchdog(request), std::invalid_argument);

  request.timeout_sec = 1.5;
  request.now_sec = nan;
  const auto resolution = update_stall_watchdog(request);
  EXPECT_FALSE(resolution.observation_accepted);
  EXPECT_FALSE(std::isfinite(resolution.stall_since_sec));
}

TEST(V2XOvertakeCoreRecovery, ArmsExtendsAndExpiresSolverCooldownAtBoundary)
{
  double cooldown_until = arm_solver_cooldown(
    SolverCooldownRequest{10.0, -std::numeric_limits<double>::infinity(), 2.0});
  EXPECT_DOUBLE_EQ(cooldown_until, 12.0);
  EXPECT_TRUE(is_solver_cooldown_active(11.999, cooldown_until));
  EXPECT_FALSE(is_solver_cooldown_active(12.0, cooldown_until));

  cooldown_until = arm_solver_cooldown(
    SolverCooldownRequest{11.0, cooldown_until, 0.5});
  EXPECT_DOUBLE_EQ(cooldown_until, 12.0);

  cooldown_until = arm_solver_cooldown(
    SolverCooldownRequest{11.0, cooldown_until, 5.0});
  EXPECT_DOUBLE_EQ(cooldown_until, 16.0);
  EXPECT_THROW(
    arm_solver_cooldown(SolverCooldownRequest{11.0, cooldown_until, -0.1}),
    std::invalid_argument);
}

TEST(V2XOvertakeCoreRecovery, RequiresCooldownAndConsecutiveSolverSuccessesForReentry)
{
  SolverReentryGateRequest request;
  request.arm = true;
  request.required_successes = 3;
  auto resolution = update_solver_reentry_gate(request);
  ASSERT_TRUE(resolution.blocked);
  EXPECT_EQ(resolution.consecutive_successes, 0);

  request.arm = false;
  request.blocked = resolution.blocked;
  request.consecutive_successes = resolution.consecutive_successes;
  request.solver_succeeded = true;
  request.cooldown_active = true;
  resolution = update_solver_reentry_gate(request);
  EXPECT_TRUE(resolution.blocked);
  EXPECT_EQ(resolution.consecutive_successes, 1);

  request.consecutive_successes = resolution.consecutive_successes;
  request.cooldown_active = false;
  resolution = update_solver_reentry_gate(request);
  EXPECT_TRUE(resolution.blocked);
  EXPECT_EQ(resolution.consecutive_successes, 2);

  request.consecutive_successes = resolution.consecutive_successes;
  resolution = update_solver_reentry_gate(request);
  EXPECT_FALSE(resolution.blocked);
  EXPECT_TRUE(resolution.released);
  EXPECT_EQ(resolution.consecutive_successes, 0);
}

TEST(V2XOvertakeCoreRecovery, SolverFailureResetsReentrySuccessCount)
{
  const auto resolution = update_solver_reentry_gate(
    SolverReentryGateRequest{false, true, 12, false, false, 20});
  EXPECT_TRUE(resolution.blocked);
  EXPECT_EQ(resolution.consecutive_successes, 0);

  EXPECT_THROW(
    update_solver_reentry_gate(
      SolverReentryGateRequest{false, true, 0, true, false, 0}),
    std::invalid_argument);
}

TEST(V2XOvertakeCoreRecovery, SaturatesReentrySuccessCountWithoutOverflow)
{
  const auto resolution = update_solver_reentry_gate(
    SolverReentryGateRequest{
      false, true, std::numeric_limits<int>::max(), true, true, 20});
  EXPECT_TRUE(resolution.blocked);
  EXPECT_FALSE(resolution.released);
  EXPECT_EQ(resolution.consecutive_successes, 20);
}

TEST(V2XOvertakeCoreRecovery, RateLimitsFallbackSteeringTowardNeutral)
{
  SolverFallbackSteeringRequest request{-0.179, 0.559, 1.2, 0.025};
  EXPECT_NEAR(
    rate_limit_solver_fallback_steering_toward_neutral(request), -0.149, 1e-12);

  request.current_steering_rad = 0.02;
  EXPECT_DOUBLE_EQ(rate_limit_solver_fallback_steering_toward_neutral(request), 0.0);

  request.current_steering_rad = 1.0;
  request.step_sec = 0.0;
  EXPECT_DOUBLE_EQ(rate_limit_solver_fallback_steering_toward_neutral(request), 0.559);

  request.steer_rate_radps = -1.0;
  EXPECT_THROW(
    rate_limit_solver_fallback_steering_toward_neutral(request), std::invalid_argument);
}

TEST(V2XOvertakeCoreRecovery, NeutralizesFallbackAfterBoundedHoldWindow)
{
  SolverFallbackNeutralizationRequest request{1, 4, false};
  EXPECT_FALSE(should_neutralize_solver_fallback_steering(request));

  request.consecutive_failures = 4;
  EXPECT_FALSE(should_neutralize_solver_fallback_steering(request));

  request.consecutive_failures = 5;
  EXPECT_TRUE(should_neutralize_solver_fallback_steering(request));

  request.consecutive_failures = 1;
  request.force_neutralize = true;
  EXPECT_TRUE(should_neutralize_solver_fallback_steering(request));

  request.consecutive_failures = -1;
  EXPECT_THROW(should_neutralize_solver_fallback_steering(request), std::invalid_argument);

  request.consecutive_failures = 0;
  request.steering_hold_cycles = -1;
  EXPECT_THROW(should_neutralize_solver_fallback_steering(request), std::invalid_argument);
}

}  // namespace
