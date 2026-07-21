#include <gtest/gtest.h>

#include <multi_purpose_mpc_ros/start_grid_grace.hpp>

#include <limits>
#include <stdexcept>

namespace start_grid_grace = ::multi_purpose_mpc_ros::start_grid_grace;

TEST(StartGridGraceGuard, DisabledDurationNeverArms) {
  start_grid_grace::Guard guard(0.0);

  EXPECT_EQ(guard.phase(), start_grid_grace::Phase::Disabled);
  EXPECT_EQ(guard.prepare(), start_grid_grace::Transition::None);
  EXPECT_EQ(guard.arm(10.0), start_grid_grace::Transition::None);
  EXPECT_FALSE(guard.evaluate(10.0).active);
}

TEST(StartGridGraceGuard, WaitingTimeDoesNotConsumeGrace) {
  start_grid_grace::Guard guard(5.0);

  EXPECT_EQ(guard.phase(), start_grid_grace::Phase::WaitingForStart);
  EXPECT_FALSE(guard.evaluate(1000.0).active);
  EXPECT_EQ(guard.arm(1000.0), start_grid_grace::Transition::Armed);
  EXPECT_TRUE(guard.evaluate(1004.999).active);
  const auto expired = guard.evaluate(1005.0);
  EXPECT_FALSE(expired.active);
  EXPECT_EQ(expired.phase, start_grid_grace::Phase::Expired);
  EXPECT_EQ(expired.transition, start_grid_grace::Transition::Expired);
}

TEST(StartGridGraceGuard,
     ReadyPreparationStaysActiveWithoutConsumingPostStartGrace) {
  start_grid_grace::Guard guard(5.0);

  EXPECT_FALSE(guard.evaluate(1000.0).active);
  EXPECT_EQ(guard.prepare(), start_grid_grace::Transition::Prepared);
  EXPECT_EQ(guard.prepare(), start_grid_grace::Transition::None);
  EXPECT_TRUE(guard.evaluate(2000.0).active);
  EXPECT_EQ(guard.arm(2000.0), start_grid_grace::Transition::Armed);
  EXPECT_TRUE(guard.evaluate(2004.999).active);
  EXPECT_FALSE(guard.evaluate(2005.0).active);
}

TEST(StartGridGraceGuard, DuplicateStartDoesNotExtendGrace) {
  start_grid_grace::Guard guard(5.0);

  EXPECT_EQ(guard.arm(20.0), start_grid_grace::Transition::Armed);
  EXPECT_EQ(guard.arm(23.0), start_grid_grace::Transition::None);
  EXPECT_FALSE(guard.evaluate(25.0).active);
}

TEST(StartGridGraceGuard, SessionClearAllowsOneNewStart) {
  start_grid_grace::Guard guard(5.0);

  ASSERT_EQ(guard.arm(10.0), start_grid_grace::Transition::Armed);
  EXPECT_EQ(guard.clear(), start_grid_grace::Transition::Cleared);
  EXPECT_EQ(guard.phase(), start_grid_grace::Phase::WaitingForStart);
  EXPECT_EQ(guard.prepare(), start_grid_grace::Transition::Prepared);
  EXPECT_EQ(guard.arm(30.0), start_grid_grace::Transition::Armed);
  EXPECT_TRUE(guard.evaluate(34.0).active);
}

TEST(StartGridGraceGuard, ClockRollbackExpiresFailClosed) {
  start_grid_grace::Guard guard(5.0);

  ASSERT_EQ(guard.arm(10.0), start_grid_grace::Transition::Armed);
  const auto rejected = guard.evaluate(9.0);
  EXPECT_FALSE(rejected.active);
  EXPECT_EQ(rejected.transition, start_grid_grace::Transition::ClockRejected);
  EXPECT_EQ(guard.phase(), start_grid_grace::Phase::Expired);
  EXPECT_EQ(guard.arm(11.0), start_grid_grace::Transition::None);
}

TEST(StartGridGraceGuard, NonFiniteTimeCannotArm) {
  start_grid_grace::Guard guard(5.0);

  EXPECT_EQ(guard.arm(std::numeric_limits<double>::quiet_NaN()),
            start_grid_grace::Transition::ClockRejected);
  EXPECT_EQ(guard.phase(), start_grid_grace::Phase::Expired);
}

TEST(StartGridGraceGuard, RejectsInvalidDuration) {
  EXPECT_THROW(start_grid_grace::Guard(-1.0), std::invalid_argument);
  EXPECT_THROW(start_grid_grace::Guard(std::numeric_limits<double>::infinity()),
               std::invalid_argument);
}

TEST(StartGridGraceSuppression, SuppressesOnlyStaticStartGridContext) {
  start_grid_grace::StaticStopContext context;
  context.grace_active = true;
  context.has_front_vehicle = true;
  context.has_side_vehicle = true;
  context.front_speed_mps = 0.0;
  context.stationary_speed_threshold_mps = 0.2;
  context.rollout_speed_threshold_mps = 1.0;

  EXPECT_FALSE(start_grid_grace::should_suppress_static_stop(context));
  context.initial_static_target_latched = true;
  EXPECT_TRUE(start_grid_grace::should_suppress_static_stop(context));
  context.has_side_vehicle = false;
  EXPECT_FALSE(start_grid_grace::should_suppress_static_stop(context));
  context.has_side_vehicle = true;
  context.front_speed_mps = 0.3;
  EXPECT_TRUE(start_grid_grace::should_suppress_static_stop(context));
  context.front_speed_mps = 1.1;
  EXPECT_FALSE(start_grid_grace::should_suppress_static_stop(context));
  context.front_speed_mps = 0.0;
  context.grace_active = false;
  EXPECT_FALSE(start_grid_grace::should_suppress_static_stop(context));
}

TEST(StartGridGraceSuppression, EmergencyBrakeAlwaysWins) {
  start_grid_grace::StaticStopContext context;
  context.grace_active = true;
  context.has_front_vehicle = true;
  context.has_side_vehicle = true;
  context.emergency_brake_required = true;
  context.initial_static_target_latched = true;
  context.front_speed_mps = 0.0;
  context.stationary_speed_threshold_mps = 0.2;
  context.rollout_speed_threshold_mps = 1.0;

  EXPECT_FALSE(start_grid_grace::should_suppress_static_stop(context));
}

TEST(StartGridGraceFrontLateralRange, UsesNormalWidthDuringGrace) {
  start_grid_grace::FrontLateralRangeContext context;
  context.grace_active = true;
  context.curve_guard_active = true;
  context.corridor_lateral_range_m = 4.0;
  context.danger_lateral_range_m = 1.55;
  context.curve_lateral_margin_m = 1.5;

  EXPECT_DOUBLE_EQ(start_grid_grace::resolve_front_lateral_range(context), 1.55);
}

TEST(StartGridGraceFrontLateralRange, RestoresCurveWidthAfterGrace) {
  start_grid_grace::FrontLateralRangeContext context;
  context.grace_active = false;
  context.curve_guard_active = true;
  context.corridor_lateral_range_m = 4.0;
  context.danger_lateral_range_m = 1.55;
  context.curve_lateral_margin_m = 1.5;

  EXPECT_DOUBLE_EQ(start_grid_grace::resolve_front_lateral_range(context), 3.05);
  context.corridor_lateral_range_m = 2.8;
  EXPECT_DOUBLE_EQ(start_grid_grace::resolve_front_lateral_range(context), 2.8);
}

TEST(StartGridGraceFrontLateralRange, KeepsNormalWidthOutsideCurves) {
  start_grid_grace::FrontLateralRangeContext context;
  context.grace_active = false;
  context.curve_guard_active = false;
  context.corridor_lateral_range_m = 4.0;
  context.danger_lateral_range_m = 1.55;
  context.curve_lateral_margin_m = 1.5;

  EXPECT_DOUBLE_EQ(start_grid_grace::resolve_front_lateral_range(context), 1.55);
}

TEST(StartGridGraceFrontLateralRange, RejectsInvalidGeometry) {
  start_grid_grace::FrontLateralRangeContext context;
  context.corridor_lateral_range_m = 4.0;
  context.danger_lateral_range_m = std::numeric_limits<double>::quiet_NaN();
  context.curve_lateral_margin_m = 1.5;

  EXPECT_THROW(start_grid_grace::resolve_front_lateral_range(context),
               std::invalid_argument);
}

TEST(StartGridGraceBreakout, AllowsOnlyLatchedStationaryGridTarget) {
  start_grid_grace::BreakoutContext context;
  context.enabled = true;
  context.grace_active = true;
  context.has_front_vehicle = true;
  context.has_side_vehicle = true;
  context.initial_static_target_latched = true;
  context.front_speed_mps = 0.0;
  context.stationary_speed_threshold_mps = 0.2;

  EXPECT_TRUE(start_grid_grace::should_attempt_breakout(context));
  context.initial_static_target_latched = false;
  EXPECT_FALSE(start_grid_grace::should_attempt_breakout(context));
  context.initial_static_target_latched = true;
  context.front_speed_mps = 0.21;
  EXPECT_FALSE(start_grid_grace::should_attempt_breakout(context));
  context.front_speed_mps = 0.0;
  context.has_side_vehicle = false;
  EXPECT_FALSE(start_grid_grace::should_attempt_breakout(context));
}

TEST(StartGridGraceBreakout, FailsClosedWhenDisabledOrInvalid) {
  start_grid_grace::BreakoutContext context;
  context.enabled = false;
  context.grace_active = true;
  context.has_front_vehicle = true;
  context.has_side_vehicle = true;
  context.initial_static_target_latched = true;
  context.front_speed_mps = 0.0;
  context.stationary_speed_threshold_mps = 0.2;

  EXPECT_FALSE(start_grid_grace::should_attempt_breakout(context));
  context.enabled = true;
  context.front_speed_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(start_grid_grace::should_attempt_breakout(context));
}

TEST(StartGridGraceBreakout, PreservesVisibleStaggeredSide) {
  start_grid_grace::BreakoutSideContext context;
  context.ego_lateral_m = -0.89;
  context.front_lateral_m = -0.07;
  context.side_deadband_m = 0.05;

  auto decision = start_grid_grace::resolve_breakout_side(context);
  ASSERT_TRUE(decision.valid);
  EXPECT_EQ(decision.required_side, -1);
  context.ego_lateral_m = 0.89;
  context.front_lateral_m = 0.07;
  decision = start_grid_grace::resolve_breakout_side(context);
  ASSERT_TRUE(decision.valid);
  EXPECT_EQ(decision.required_side, 1);
}

TEST(StartGridGraceBreakout, LetsGapPlannerChooseWhenStaggerIsInsideDeadband) {
  start_grid_grace::BreakoutSideContext context;
  context.ego_lateral_m = 0.03;
  context.front_lateral_m = 0.0;
  context.side_deadband_m = 0.05;

  const auto decision = start_grid_grace::resolve_breakout_side(context);
  ASSERT_TRUE(decision.valid);
  EXPECT_EQ(decision.required_side, 0);
}

TEST(StartGridGraceBreakout, KeepsLatchedSideWhenRelativeLateralPositionChanges) {
  start_grid_grace::BreakoutSideContext context;
  context.ego_lateral_m = 0.80;
  context.front_lateral_m = 0.0;
  context.side_deadband_m = 0.05;
  context.latched_side = -1;

  auto decision = start_grid_grace::resolve_breakout_side(context);
  ASSERT_TRUE(decision.valid);
  EXPECT_EQ(decision.required_side, -1);

  context.ego_lateral_m = -0.80;
  context.front_lateral_m = 0.20;
  decision = start_grid_grace::resolve_breakout_side(context);
  ASSERT_TRUE(decision.valid);
  EXPECT_EQ(decision.required_side, -1);
}

TEST(StartGridGraceBreakout, RejectsInvalidSideGeometry) {
  start_grid_grace::BreakoutSideContext context;
  context.ego_lateral_m = std::numeric_limits<double>::quiet_NaN();
  context.front_lateral_m = 0.0;
  context.side_deadband_m = 0.05;

  EXPECT_FALSE(start_grid_grace::resolve_breakout_side(context).valid);
  context.ego_lateral_m = 1.0;
  context.side_deadband_m = -0.1;
  EXPECT_FALSE(start_grid_grace::resolve_breakout_side(context).valid);
  context.side_deadband_m = 0.05;
  context.latched_side = 2;
  EXPECT_FALSE(start_grid_grace::resolve_breakout_side(context).valid);
}
