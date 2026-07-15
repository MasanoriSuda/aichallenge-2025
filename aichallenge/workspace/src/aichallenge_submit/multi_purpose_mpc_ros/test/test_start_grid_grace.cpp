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
