#include "multi_purpose_mpc_ros/awsim_control_mode_guard.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace
{

using multi_purpose_mpc_ros::awsim_control_mode::Config;
using multi_purpose_mpc_ros::awsim_control_mode::Event;
using multi_purpose_mpc_ros::awsim_control_mode::LaunchEngagementGuard;
using multi_purpose_mpc_ros::awsim_control_mode::Phase;

Config enabled_config()
{
  Config config;
  config.enabled = true;
  config.retry_period_sec = 0.2;
  config.retry_timeout_sec = 5.0;
  config.motion_speed_threshold_mps = 0.1;
  return config;
}

TEST(AwsimControlModeGuard, DisabledNeverRequestsOrSuppressesRecovery)
{
  Config config;
  LaunchEngagementGuard guard(config);

  EXPECT_FALSE(guard.on_awsim_state("Ready", 1.0).publish_request);
  EXPECT_FALSE(guard.on_awsim_state("Start", 2.0).publish_request);
  EXPECT_FALSE(guard.update(2.5, 0.0).suppress_stuck_recovery);
  EXPECT_EQ(guard.phase(), Phase::Disabled);
}

TEST(AwsimControlModeGuard, ReadyRequestsAndStartsBoundedRetry)
{
  LaunchEngagementGuard guard(enabled_config());

  const auto decision = guard.on_awsim_state(" Ready ", 1.0);
  EXPECT_EQ(decision.event, Event::ReadyRequest);
  EXPECT_TRUE(decision.publish_request);
  EXPECT_TRUE(decision.suppress_stuck_recovery);
  EXPECT_EQ(guard.phase(), Phase::AwaitingMotion);
  EXPECT_EQ(guard.update(1.2, 0.0).event, Event::RetryRequest);
}

TEST(AwsimControlModeGuard, StartRequestsAndRetriesUntilMotion)
{
  LaunchEngagementGuard guard(enabled_config());

  const auto start = guard.on_awsim_state("START", 10.0);
  EXPECT_EQ(start.event, Event::StartRequest);
  EXPECT_TRUE(start.publish_request);
  EXPECT_TRUE(start.suppress_stuck_recovery);
  EXPECT_FALSE(guard.update(10.19, 0.0).publish_request);

  const auto retry = guard.update(10.20, 0.0);
  EXPECT_EQ(retry.event, Event::RetryRequest);
  EXPECT_TRUE(retry.publish_request);
  EXPECT_TRUE(retry.suppress_stuck_recovery);

  const auto moving = guard.update(10.21, 0.1);
  EXPECT_EQ(moving.event, Event::MotionConfirmed);
  EXPECT_FALSE(moving.publish_request);
  EXPECT_FALSE(moving.suppress_stuck_recovery);
  EXPECT_EQ(guard.phase(), Phase::MotionConfirmed);
  EXPECT_FALSE(guard.update(11.0, 0.0).publish_request);
}

TEST(AwsimControlModeGuard, TimesOutAndReenablesStuckRecovery)
{
  LaunchEngagementGuard guard(enabled_config());
  guard.on_awsim_state("start", 20.0);

  const auto timeout = guard.update(25.0, 0.0);
  EXPECT_EQ(timeout.event, Event::TimedOut);
  EXPECT_FALSE(timeout.publish_request);
  EXPECT_FALSE(timeout.suppress_stuck_recovery);
  EXPECT_EQ(guard.phase(), Phase::TimedOut);
}

TEST(AwsimControlModeGuard, StartRearmsAfterReadyWindowTimesOut)
{
  LaunchEngagementGuard guard(enabled_config());
  guard.on_awsim_state("ready", 1.0);
  ASSERT_EQ(guard.update(6.0, 0.0).event, Event::TimedOut);

  const auto start = guard.on_awsim_state("start", 10.0);
  EXPECT_EQ(start.event, Event::StartRequest);
  EXPECT_TRUE(start.publish_request);
  EXPECT_TRUE(start.suppress_stuck_recovery);
  EXPECT_EQ(guard.update(10.2, 0.0).event, Event::RetryRequest);
}

TEST(AwsimControlModeGuard, SessionBoundaryResetsTheGuard)
{
  LaunchEngagementGuard guard(enabled_config());
  guard.on_awsim_state("start", 1.0);
  ASSERT_TRUE(guard.suppress_stuck_recovery());

  const auto reset = guard.on_awsim_state("Finish", 2.0);
  EXPECT_EQ(reset.event, Event::Reset);
  EXPECT_EQ(guard.phase(), Phase::Idle);
  EXPECT_FALSE(guard.suppress_stuck_recovery());

  EXPECT_TRUE(guard.on_awsim_state("ready", 3.0).publish_request);
  EXPECT_TRUE(guard.on_awsim_state("start", 4.0).publish_request);
}

TEST(AwsimControlModeGuard, ClockRollbackFailsOpenForRecovery)
{
  LaunchEngagementGuard guard(enabled_config());
  guard.on_awsim_state("start", 10.0);

  const auto rollback = guard.update(9.0, 0.0);
  EXPECT_EQ(rollback.event, Event::TimedOut);
  EXPECT_FALSE(rollback.suppress_stuck_recovery);
}

TEST(AwsimControlModeGuard, RejectsInvalidConfiguration)
{
  auto config = enabled_config();
  config.retry_period_sec = 0.0;
  EXPECT_THROW(LaunchEngagementGuard{config}, std::invalid_argument);

  config = enabled_config();
  config.retry_timeout_sec = std::numeric_limits<double>::infinity();
  EXPECT_THROW(LaunchEngagementGuard{config}, std::invalid_argument);

  config = enabled_config();
  config.motion_speed_threshold_mps = -0.1;
  EXPECT_THROW(LaunchEngagementGuard{config}, std::invalid_argument);
}

}  // namespace
