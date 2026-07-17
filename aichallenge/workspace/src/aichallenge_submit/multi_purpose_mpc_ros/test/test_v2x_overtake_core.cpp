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
using multi_purpose_mpc_ros::v2x_overtake_core::ContinuityAction;
using multi_purpose_mpc_ros::v2x_overtake_core::ContinuityRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::CoursePoint;
using multi_purpose_mpc_ros::v2x_overtake_core::ForwardDistanceRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::FrontHazardHoldRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::ForwardCourseProjectionRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::OvertakeSpeedReferenceRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::OvertakeSpeedStage;
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
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_overtake_speed_reference;
using multi_purpose_mpc_ros::v2x_overtake_core::advance_prediction_time;
using multi_purpose_mpc_ros::v2x_overtake_core::project_forward_course_progress;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_pass_completion;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_target_continuity;
using multi_purpose_mpc_ros::v2x_overtake_core::can_reacquire_during_return;
using multi_purpose_mpc_ros::v2x_overtake_core::integrate_forward_distance;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_recovery_policy;
using multi_purpose_mpc_ros::v2x_overtake_core::update_stall_watchdog;
using multi_purpose_mpc_ros::v2x_overtake_core::update_front_hazard_hold;
using multi_purpose_mpc_ros::v2x_overtake_core::arm_solver_cooldown;
using multi_purpose_mpc_ros::v2x_overtake_core::is_solver_cooldown_active;
using multi_purpose_mpc_ros::v2x_overtake_core::rate_limit_solver_fallback_steering_toward_neutral;
using multi_purpose_mpc_ros::v2x_overtake_core::should_neutralize_solver_fallback_steering;
using multi_purpose_mpc_ros::v2x_overtake_core::update_solver_reentry_gate;
using multi_purpose_mpc_ros::v2x_overtake_core::select_pass_side;

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

TEST(V2XOvertakeCoreSpeed, UsesCappedNormalSpeedWithoutStartConfiguration)
{
  auto request = speed_request();
  request.normal_speed_mps = 50.0;
  const auto result = resolve_effective_speed_limit(request);
  EXPECT_DOUBLE_EQ(result.speed_mps, 40.0);
  EXPECT_EQ(result.start_window_status, StartWindowStatus::NotConfigured);
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

TEST(V2XFrontHazardHold, RejectsInvalidClockAndDuration)
{
  FrontHazardHoldRequest request{true, false, false, 1.0, 1.0, -0.1};
  EXPECT_THROW(update_front_hazard_hold(request), std::invalid_argument);
  request.hold_sec = 1.0;
  request.now_sec = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(update_front_hazard_hold(request), std::invalid_argument);
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

TEST(V2XOvertakeCoreContinuity, HoldsShortTargetLossInsteadOfReturning)
{
  ContinuityRequest request;
  request.target_age_sec = 0.10;
  request.target_hold_sec = 0.30;
  EXPECT_EQ(resolve_target_continuity(request), ContinuityAction::Hold);

  request.target_age_sec = 0.31;
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
