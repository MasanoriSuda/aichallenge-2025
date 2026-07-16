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
using multi_purpose_mpc_ros::awsim_boost::StateEvent;
using multi_purpose_mpc_ros::awsim_boost::StartDashGuard;
using multi_purpose_mpc_ros::awsim_boost::Trigger;
using multi_purpose_mpc_ros::awsim_boost::TriggerContext;

Config enabled_config()
{
  Config config;
  config.enabled = true;
  config.mode = Mode::StartOnce;
  config.trigger = Trigger::AwsimStart;
  config.status_timeout_sec = 0.5;
  config.confirmation_timeout_sec = 2.0;
  return config;
}

Config motion_config()
{
  auto config = enabled_config();
  config.trigger = Trigger::FirstForwardMotion;
  config.motion_speed_threshold_mps = 0.1;
  config.max_trigger_speed_mps = 1.0;
  config.motion_trigger_timeout_sec = 0.5;
  return config;
}

TriggerContext healthy_motion_context(const double forward_speed_mps)
{
  TriggerContext context;
  context.control_enabled = true;
  context.normal_command_published = true;
  context.forward_speed_mps = forward_speed_mps;
  return context;
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
  EXPECT_EQ(guard.on_awsim_state("Start"), StateEvent::StartEntered);
  EXPECT_FALSE(guard.on_awsim_status(status(2.0F, 0.0F), now));
  const auto result = guard.evaluate(true, false, now);
  EXPECT_EQ(result.action, Action::None);
  EXPECT_EQ(result.reason, BlockReason::Disabled);
  EXPECT_EQ(guard.phase(), Phase::Disabled);
}

TEST(AwsimBoostStartDash, ReportsRaceSessionEdgesEvenWhenBoostIsDisabled)
{
  Config config;
  config.enabled = false;
  config.mode = Mode::Disabled;
  StartDashGuard guard(config);
  const auto now = StartDashGuard::Clock::now();

  EXPECT_EQ(guard.on_awsim_state(" Spawned "), StateEvent::None);
  EXPECT_EQ(guard.on_awsim_state("Start"), StateEvent::StartEntered);
  EXPECT_EQ(guard.on_awsim_state("start"), StateEvent::None);
  EXPECT_EQ(guard.on_awsim_state("Ready"), StateEvent::None);
  EXPECT_EQ(guard.on_awsim_state("START"), StateEvent::None);

  EXPECT_EQ(guard.on_awsim_state("Finish"), StateEvent::Finished);
  EXPECT_EQ(guard.on_awsim_state("finish"), StateEvent::None);
  EXPECT_EQ(guard.on_awsim_state("Start"), StateEvent::None);
  EXPECT_EQ(guard.on_awsim_state("Spawned"), StateEvent::NewSession);
  EXPECT_EQ(guard.on_awsim_state("spawned"), StateEvent::None);
  EXPECT_EQ(guard.phase(), Phase::Disabled);

  EXPECT_EQ(guard.on_awsim_state("Start"), StateEvent::StartEntered);
  EXPECT_FALSE(guard.on_awsim_status(status(5.0F, 0.0F), now));
  const auto evaluation = guard.evaluate(true, false, now);
  EXPECT_EQ(evaluation.action, Action::None);
  EXPECT_EQ(evaluation.reason, BlockReason::Disabled);
}

TEST(AwsimBoostStartDash, FinishBlocksBoostUntilSpawnedStartsNewSession)
{
  StartDashGuard guard(enabled_config());
  const auto now = StartDashGuard::Clock::now();
  EXPECT_EQ(guard.on_awsim_state("Start"), StateEvent::StartEntered);
  ASSERT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), now));

  EXPECT_EQ(guard.on_awsim_state("Finish"), StateEvent::Finished);
  EXPECT_EQ(guard.on_awsim_state("Start"), StateEvent::None);
  EXPECT_EQ(guard.evaluate(true, false, now).reason, BlockReason::AwaitingStart);

  EXPECT_EQ(guard.on_awsim_state("Spawned"), StateEvent::NewSession);
  EXPECT_EQ(guard.on_awsim_state("Start"), StateEvent::StartEntered);
  ASSERT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), now));
  EXPECT_EQ(guard.evaluate(true, false, now).action, Action::PublishPulse);
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

TEST(AwsimBoostStartDash, FirstForwardMotionWaitsForReadyAndMotion)
{
  StartDashGuard guard(motion_config());
  const auto now = StartDashGuard::Clock::now();
  ASSERT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), now));

  EXPECT_EQ(
    guard.evaluate(healthy_motion_context(0.0), now).reason,
    BlockReason::AwaitingReady);
  EXPECT_EQ(guard.on_awsim_state("Ready"), StateEvent::None);
  EXPECT_EQ(guard.phase(), Phase::AwaitingMotion);
  EXPECT_EQ(
    guard.evaluate(healthy_motion_context(0.0), now).reason,
    BlockReason::AwaitingMotion);
  EXPECT_EQ(
    guard.evaluate(healthy_motion_context(0.099), now).reason,
    BlockReason::AwaitingMotion);

  const auto launched = guard.evaluate(
    healthy_motion_context(0.1), now + std::chrono::milliseconds(10));
  EXPECT_EQ(launched.action, Action::PublishPulse);
  EXPECT_TRUE(launched.motion_detected_now);
  EXPECT_DOUBLE_EQ(launched.motion_elapsed_sec, 0.0);
  EXPECT_EQ(guard.phase(), Phase::PulseSent);
}

TEST(AwsimBoostStartDash, MotionTriggerWaitsForSafetyThenPublishesWithinWindow)
{
  StartDashGuard guard(motion_config());
  const auto now = StartDashGuard::Clock::now();
  guard.on_awsim_state("ready");
  ASSERT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), now));

  auto context = healthy_motion_context(0.1);
  context.v2x_safety_brake_active = true;
  const auto safety_blocked = guard.evaluate(context, now);
  EXPECT_TRUE(safety_blocked.motion_detected_now);
  EXPECT_EQ(safety_blocked.reason, BlockReason::SafetyBrakeActive);
  EXPECT_EQ(guard.phase(), Phase::MotionDetected);

  context.v2x_safety_brake_active = false;
  context.solver_fallback_active = true;
  EXPECT_EQ(
    guard.evaluate(context, now + std::chrono::milliseconds(100)).reason,
    BlockReason::SolverFallbackActive);
  context.solver_fallback_active = false;
  const auto published = guard.evaluate(context, now + std::chrono::milliseconds(200));
  EXPECT_EQ(published.action, Action::PublishPulse);
  EXPECT_NEAR(published.motion_elapsed_sec, 0.2, 1e-9);
}

TEST(AwsimBoostStartDash, MotionTriggerExposesAllControlInhibits)
{
  StartDashGuard guard(motion_config());
  const auto now = StartDashGuard::Clock::now();
  guard.on_awsim_state("ready");
  ASSERT_TRUE(guard.on_awsim_status(status(2.0F, 0.0F), now));

  auto context = healthy_motion_context(0.1);
  context.normal_command_published = false;
  EXPECT_EQ(guard.evaluate(context, now).reason, BlockReason::CommandNotPublished);
  context.normal_command_published = true;
  context.reverse_or_recovery_active = true;
  EXPECT_EQ(
    guard.evaluate(context, now + std::chrono::milliseconds(25)).reason,
    BlockReason::ReverseOrRecoveryActive);
  context.reverse_or_recovery_active = false;
  context.failsafe_active = true;
  EXPECT_EQ(
    guard.evaluate(context, now + std::chrono::milliseconds(50)).reason,
    BlockReason::FailsafeActive);
  context.failsafe_active = false;
  EXPECT_EQ(
    guard.evaluate(context, now + std::chrono::milliseconds(75)).action,
    Action::PublishPulse);
}

TEST(AwsimBoostStartDash, MotionTriggerTimeoutAndSpeedCeilingNeverRetry)
{
  const auto now = StartDashGuard::Clock::now();
  StartDashGuard timed_out(motion_config());
  timed_out.on_awsim_state("ready");
  ASSERT_TRUE(timed_out.on_awsim_status(status(2.0F, 0.0F), now));
  auto blocked = healthy_motion_context(0.1);
  blocked.control_enabled = false;
  EXPECT_EQ(timed_out.evaluate(blocked, now).reason, BlockReason::ControlDisabled);
  timed_out.on_awsim_state("Ready");
  EXPECT_EQ(
    timed_out.evaluate(blocked, now + std::chrono::milliseconds(501)).reason,
    BlockReason::MotionTriggerTimedOut);
  EXPECT_EQ(timed_out.phase(), Phase::LaunchExpiredSpent);
  EXPECT_EQ(
    timed_out.evaluate(healthy_motion_context(0.1), now + std::chrono::seconds(1)).reason,
    BlockReason::AlreadySpent);

  StartDashGuard too_fast(motion_config());
  too_fast.on_awsim_state("ready");
  ASSERT_TRUE(too_fast.on_awsim_status(status(2.0F, 0.0F), now));
  EXPECT_EQ(
    too_fast.evaluate(healthy_motion_context(1.01), now).reason,
    BlockReason::MotionTriggerSpeedExceeded);
  EXPECT_EQ(too_fast.phase(), Phase::LaunchExpiredSpent);
}

TEST(AwsimBoostStartDash, StartFallbackAcceptsLowSpeedButRejectsDelayedHighSpeed)
{
  const auto now = StartDashGuard::Clock::now();
  StartDashGuard low_speed(motion_config());
  EXPECT_EQ(low_speed.on_awsim_state("start"), StateEvent::StartEntered);
  ASSERT_TRUE(low_speed.on_awsim_status(status(2.0F, 0.0F), now));
  EXPECT_EQ(
    low_speed.evaluate(healthy_motion_context(0.2), now).action,
    Action::PublishPulse);

  StartDashGuard delayed(motion_config());
  EXPECT_EQ(delayed.on_awsim_state("start"), StateEvent::StartEntered);
  ASSERT_TRUE(delayed.on_awsim_status(status(2.0F, 0.0F), now));
  EXPECT_EQ(
    delayed.evaluate(healthy_motion_context(2.0), now).reason,
    BlockReason::MotionTriggerSpeedExceeded);
}

TEST(AwsimBoostStartDash, MotionTriggerRejectsNonfiniteForwardSpeed)
{
  StartDashGuard guard(motion_config());
  const auto now = StartDashGuard::Clock::now();
  guard.on_awsim_state("ready");
  auto context = healthy_motion_context(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(guard.evaluate(context, now).reason, BlockReason::InvalidForwardSpeed);
  EXPECT_EQ(guard.phase(), Phase::AwaitingMotion);
}

TEST(AwsimBoostStartDash, ParsesOnlySupportedModes)
{
  using multi_purpose_mpc_ros::awsim_boost::parse_mode;
  EXPECT_EQ(parse_mode(" disabled "), Mode::Disabled);
  EXPECT_EQ(parse_mode("START_ONCE"), Mode::StartOnce);
  EXPECT_THROW(parse_mode("straight_only"), std::invalid_argument);
}

TEST(AwsimBoostStartDash, ParsesOnlySupportedTriggers)
{
  using multi_purpose_mpc_ros::awsim_boost::parse_trigger;
  EXPECT_EQ(parse_trigger(" awsim_start "), Trigger::AwsimStart);
  EXPECT_EQ(parse_trigger("FIRST_FORWARD_MOTION"), Trigger::FirstForwardMotion);
  EXPECT_THROW(parse_trigger("ready"), std::invalid_argument);
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
  config = motion_config();
  config.motion_speed_threshold_mps = 0.0;
  EXPECT_THROW(StartDashGuard guard(config), std::invalid_argument);
  config = motion_config();
  config.max_trigger_speed_mps = 0.05;
  EXPECT_THROW(StartDashGuard guard(config), std::invalid_argument);
  config = motion_config();
  config.motion_trigger_timeout_sec = -1.0;
  EXPECT_THROW(StartDashGuard guard(config), std::invalid_argument);
}

}  // namespace
