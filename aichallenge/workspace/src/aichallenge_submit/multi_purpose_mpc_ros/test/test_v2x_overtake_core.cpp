#include "multi_purpose_mpc_ros/v2x_overtake_core.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <optional>
#include <stdexcept>

namespace
{

using multi_purpose_mpc_ros::v2x_overtake_core::PassSide;
using multi_purpose_mpc_ros::v2x_overtake_core::ContinuityAction;
using multi_purpose_mpc_ros::v2x_overtake_core::ContinuityRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::ReacquireRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::SideSelectionReason;
using multi_purpose_mpc_ros::v2x_overtake_core::SideSelectionRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::SpeedLimitRequest;
using multi_purpose_mpc_ros::v2x_overtake_core::StartWindowStatus;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_effective_speed_limit;
using multi_purpose_mpc_ros::v2x_overtake_core::resolve_target_continuity;
using multi_purpose_mpc_ros::v2x_overtake_core::can_reacquire_during_return;
using multi_purpose_mpc_ros::v2x_overtake_core::select_pass_side;

SpeedLimitRequest speed_request()
{
  SpeedLimitRequest request;
  request.normal_speed_mps = 20.0;
  request.global_hard_cap_mps = 40.0;
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

}  // namespace
