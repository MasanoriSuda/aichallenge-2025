#include "multi_purpose_mpc_ros/awsim_boost_start_dash.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

namespace
{

using multi_purpose_mpc_ros::awsim_boost::Action;
using multi_purpose_mpc_ros::awsim_boost::BlockReason;
using multi_purpose_mpc_ros::awsim_boost::Config;
using multi_purpose_mpc_ros::awsim_boost::Mode;
using multi_purpose_mpc_ros::awsim_boost::Phase;
using multi_purpose_mpc_ros::awsim_boost::StartDashGuard;

Config enabled_config()
{
  Config config;
  config.enabled = true;
  config.mode = Mode::StartOnce;
  config.status_timeout_sec = 0.5;
  config.confirmation_timeout_sec = 2.0;
  return config;
}

std::vector<float> status(const float remaining, const float is_boosting)
{
  return {100.0F, 0.0F, 0.0F, 0.0F, 1.0F, remaining, is_boosting};
}

TEST(AwsimBoostStartDash, DisabledModeNeverTriggers)
{
  auto config = enabled_config();
  config.mode = Mode::Disabled;
  StartDashGuard guard(config);
  const auto now = StartDashGuard::Clock::now();
  guard.on_awsim_state("Start");
  EXPECT_FALSE(guard.on_awsim_status(status(2.0F, 0.0F), now));
  const auto result = guard.evaluate(true, false, now);
  EXPECT_EQ(result.action, Action::None);
  EXPECT_EQ(result.reason, BlockReason::Disabled);
  EXPECT_EQ(guard.phase(), Phase::Disabled);
}

TEST(AwsimBoostStartDash, RequiresStartAndFreshStatus)
{
  StartDashGuard guard(enabled_config());
  const auto now = StartDashGuard::Clock::now();
  EXPECT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), now));
  EXPECT_EQ(guard.evaluate(true, false, now).reason, BlockReason::AwaitingStart);

  guard.on_awsim_state(" Start ");
  const auto result = guard.evaluate(true, false, now + std::chrono::milliseconds(10));
  EXPECT_EQ(result.action, Action::PublishPulse);
  EXPECT_EQ(guard.phase(), Phase::PulseSent);
}

TEST(AwsimBoostStartDash, InvalidStatusCannotReuseAnOlderValidSample)
{
  StartDashGuard guard(enabled_config());
  const auto now = StartDashGuard::Clock::now();
  guard.on_awsim_state("start");
  ASSERT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), now));
  EXPECT_FALSE(guard.on_awsim_status({1.0F, 2.0F}, now + std::chrono::milliseconds(1)));
  EXPECT_FALSE(guard.has_valid_status());
  EXPECT_EQ(
    guard.evaluate(true, false, now + std::chrono::milliseconds(2)).reason,
    BlockReason::MissingStatus);

  auto nonfinite = status(2.0F, 0.0F);
  nonfinite[5] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(guard.on_awsim_status(nonfinite, now + std::chrono::milliseconds(3)));
  EXPECT_EQ(
    guard.evaluate(true, false, now + std::chrono::milliseconds(4)).reason,
    BlockReason::MissingStatus);
}

TEST(AwsimBoostStartDash, RejectsStaleOrFutureStatus)
{
  StartDashGuard guard(enabled_config());
  const auto now = StartDashGuard::Clock::now();
  guard.on_awsim_state("start");
  ASSERT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), now));
  EXPECT_EQ(
    guard.evaluate(true, false, now + std::chrono::milliseconds(501)).reason,
    BlockReason::StaleStatus);
  EXPECT_EQ(
    guard.evaluate(true, false, now - std::chrono::milliseconds(1)).reason,
    BlockReason::StaleStatus);
}

TEST(AwsimBoostStartDash, RequiresRemainingBoostAndNonBoostingState)
{
  const auto now = StartDashGuard::Clock::now();
  StartDashGuard no_remaining(enabled_config());
  no_remaining.on_awsim_state("start");
  ASSERT_TRUE(no_remaining.on_awsim_status(status(0.0F, 0.0F), now));
  EXPECT_EQ(
    no_remaining.evaluate(true, false, now).reason,
    BlockReason::NoRemainingBoost);

  StartDashGuard already_boosting(enabled_config());
  already_boosting.on_awsim_state("start");
  ASSERT_TRUE(already_boosting.on_awsim_status(status(2.0F, 1.0F), now));
  EXPECT_EQ(
    already_boosting.evaluate(true, false, now).reason,
    BlockReason::AlreadyBoosting);
}

TEST(AwsimBoostStartDash, WaitsForHealthyEnabledControl)
{
  StartDashGuard guard(enabled_config());
  const auto now = StartDashGuard::Clock::now();
  guard.on_awsim_state("start");
  ASSERT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), now));
  EXPECT_EQ(guard.evaluate(false, false, now).reason, BlockReason::ControlDisabled);
  EXPECT_EQ(guard.evaluate(true, true, now).reason, BlockReason::FailsafeActive);
  EXPECT_EQ(guard.evaluate(true, false, now).action, Action::PublishPulse);
}

TEST(AwsimBoostStartDash, EmitsOnlyOneActionPerSession)
{
  StartDashGuard guard(enabled_config());
  const auto now = StartDashGuard::Clock::now();
  guard.on_awsim_state("start");
  ASSERT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), now));
  ASSERT_EQ(guard.evaluate(true, false, now).action, Action::PublishPulse);

  for (int i = 1; i <= 100; ++i) {
    const auto tick = now + std::chrono::milliseconds(i * 10);
    ASSERT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), tick));
    EXPECT_EQ(guard.evaluate(true, false, tick).action, Action::None);
  }
}

TEST(AwsimBoostStartDash, ConfirmsFromBoostingFlagOrRemainingDecrease)
{
  const auto now = StartDashGuard::Clock::now();
  StartDashGuard boosting_flag(enabled_config());
  boosting_flag.on_awsim_state("start");
  ASSERT_TRUE(boosting_flag.on_awsim_status(status(2.0F, 0.0F), now));
  ASSERT_EQ(boosting_flag.evaluate(true, false, now).action, Action::PublishPulse);
  ASSERT_TRUE(boosting_flag.on_awsim_status(
      status(2.0F, 1.0F), now + std::chrono::milliseconds(10)));
  EXPECT_EQ(boosting_flag.phase(), Phase::Confirmed);

  StartDashGuard remaining_decrease(enabled_config());
  remaining_decrease.on_awsim_state("start");
  ASSERT_TRUE(remaining_decrease.on_awsim_status(status(2.0F, 0.0F), now));
  ASSERT_EQ(remaining_decrease.evaluate(true, false, now).action, Action::PublishPulse);
  ASSERT_TRUE(remaining_decrease.on_awsim_status(
      status(1.0F, 0.0F), now + std::chrono::milliseconds(10)));
  EXPECT_EQ(remaining_decrease.phase(), Phase::Confirmed);
}

TEST(AwsimBoostStartDash, ConfirmationTimeoutNeverRetries)
{
  StartDashGuard guard(enabled_config());
  const auto now = StartDashGuard::Clock::now();
  guard.on_awsim_state("start");
  ASSERT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), now));
  ASSERT_EQ(guard.evaluate(true, false, now).action, Action::PublishPulse);

  const auto timeout = now + std::chrono::seconds(2);
  const auto timed_out = guard.evaluate(true, false, timeout);
  EXPECT_EQ(timed_out.action, Action::None);
  EXPECT_EQ(timed_out.reason, BlockReason::ConfirmationTimedOut);
  EXPECT_EQ(guard.phase(), Phase::UnconfirmedSpent);
  EXPECT_EQ(
    guard.evaluate(true, false, timeout + std::chrono::seconds(1)).reason,
    BlockReason::AlreadySpent);
}

TEST(AwsimBoostStartDash, DuplicateStatesDoNotRearmSpentSession)
{
  StartDashGuard guard(enabled_config());
  const auto now = StartDashGuard::Clock::now();
  guard.on_awsim_state("start");
  ASSERT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), now));
  ASSERT_EQ(guard.evaluate(true, false, now).action, Action::PublishPulse);

  guard.on_awsim_state("Start");
  guard.on_awsim_state("Ready");
  guard.on_awsim_state("Spawned");
  ASSERT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), now + std::chrono::milliseconds(10)));
  EXPECT_EQ(
    guard.evaluate(true, false, now + std::chrono::milliseconds(10)).action,
    Action::None);
  EXPECT_EQ(guard.phase(), Phase::PulseSent);
}

TEST(AwsimBoostStartDash, FinishThenSpawnedRearmsNextSessionOnce)
{
  StartDashGuard guard(enabled_config());
  const auto now = StartDashGuard::Clock::now();
  guard.on_awsim_state("start");
  ASSERT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), now));
  ASSERT_EQ(guard.evaluate(true, false, now).action, Action::PublishPulse);
  ASSERT_TRUE(guard.on_awsim_status(status(1.0F, 1.0F), now + std::chrono::milliseconds(10)));
  ASSERT_EQ(guard.phase(), Phase::Confirmed);

  guard.on_awsim_state("finish");
  guard.on_awsim_state("spawned");
  EXPECT_EQ(guard.phase(), Phase::Armed);
  guard.on_awsim_state("start");
  ASSERT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), now + std::chrono::seconds(1)));
  EXPECT_EQ(
    guard.evaluate(true, false, now + std::chrono::seconds(1)).action,
    Action::PublishPulse);
  EXPECT_EQ(
    guard.evaluate(true, false, now + std::chrono::seconds(1)).action,
    Action::None);
}

TEST(AwsimBoostStartDash, ParsesOnlySupportedModes)
{
  using multi_purpose_mpc_ros::awsim_boost::parse_mode;
  EXPECT_EQ(parse_mode(" disabled "), Mode::Disabled);
  EXPECT_EQ(parse_mode("START_ONCE"), Mode::StartOnce);
  EXPECT_THROW(parse_mode("straight_only"), std::invalid_argument);
}

TEST(AwsimBoostStartDash, ResolvesDomainEnabledOverrideWithGlobalFallback)
{
  using multi_purpose_mpc_ros::awsim_boost::resolve_enabled;
  const std::map<int, bool> domain_enabled{{1, true}, {2, false}};

  const auto domain_one = resolve_enabled(false, domain_enabled, 1);
  EXPECT_TRUE(domain_one.enabled);
  EXPECT_TRUE(domain_one.domain_override_applied);
  EXPECT_EQ(domain_one.domain_id, 1);

  const auto domain_two = resolve_enabled(true, domain_enabled, 2);
  EXPECT_FALSE(domain_two.enabled);
  EXPECT_TRUE(domain_two.domain_override_applied);
  EXPECT_EQ(domain_two.domain_id, 2);

  const auto unknown_domain = resolve_enabled(true, domain_enabled, 3);
  EXPECT_TRUE(unknown_domain.enabled);
  EXPECT_FALSE(unknown_domain.domain_override_applied);
  EXPECT_EQ(unknown_domain.domain_id, 3);

  const auto missing_domain = resolve_enabled(false, domain_enabled, std::nullopt);
  EXPECT_FALSE(missing_domain.enabled);
  EXPECT_FALSE(missing_domain.domain_override_applied);
  EXPECT_EQ(missing_domain.domain_id, -1);
}

TEST(AwsimBoostStartDash, OfficialCommandPulseIsOneThenZero)
{
  using multi_purpose_mpc_ros::awsim_boost::kCommandPulseValues;
  EXPECT_FLOAT_EQ(kCommandPulseValues[0], 1.0F);
  EXPECT_FLOAT_EQ(kCommandPulseValues[1], 0.0F);
}

TEST(AwsimBoostStartDash, RejectsInvalidTimeouts)
{
  auto config = enabled_config();
  config.status_timeout_sec = 0.0;
  EXPECT_THROW(StartDashGuard guard(config), std::invalid_argument);
  config = enabled_config();
  config.confirmation_timeout_sec = std::numeric_limits<double>::infinity();
  EXPECT_THROW(StartDashGuard guard(config), std::invalid_argument);
}

}  // namespace
