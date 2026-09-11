#include "multi_purpose_mpc_ros/mpcc_publication_appointment.hpp"

#include <gtest/gtest.h>
#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

namespace {
using Alarm = multi_purpose_mpc_ros::mpcc_publication_appointment::Alarm;

class PublicationAppointment : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("mpcc_appointment_test");
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ASSERT_EQ(rcl_enable_ros_time_override(clock_->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_->get_clock_handle(), 0), RCL_RET_OK);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    alarm_ = std::make_unique<Alarm>(node_->get_node_base_interface(),
      node_->get_node_timers_interface(), clock_, [this](std::int64_t appointment) {
        calls_.push_back(appointment);
        actual_.push_back(clock_->now().nanoseconds());
        if (rearm_from_callback_) alarm_->arm(appointment + 25000000);
      });
  }
  void TearDown() override {
    alarm_.reset();
    executor_->remove_node(node_);
    executor_.reset();
    node_.reset();
    clock_.reset();
  }
  void advance(std::int64_t ns) {
    ASSERT_EQ(rcl_set_ros_time_override(clock_->get_clock_handle(), ns), RCL_RET_OK);
    executor_->spin_some();
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<Alarm> alarm_;
  std::vector<std::int64_t> calls_, actual_;
  bool rearm_from_callback_{false};
};

TEST_F(PublicationAppointment, SavedR49DoesNotWake67NanosecondsBeforeOriginalOpening) {
  constexpr std::int64_t before = 12164999728LL, opening = 12164999795LL;
  advance(before);
  alarm_->arm(opening);
  advance(before);
  EXPECT_TRUE(calls_.empty());
  advance(opening - 1);
  EXPECT_TRUE(calls_.empty());
  advance(opening);
  ASSERT_EQ(calls_.size(), 1U);
  EXPECT_EQ(calls_.front(), opening);
  EXPECT_EQ(actual_.front(), opening);
  advance(opening + 1);
  EXPECT_EQ(calls_.size(), 1U);
}

TEST_F(PublicationAppointment, DiscreteClockTicksPreserveTheOriginal25MillisecondSequence) {
  constexpr std::int64_t opening = 12164999795LL;
  advance(opening - 67);
  rearm_from_callback_ = true;
  alarm_->arm(opening);
  // The observed clock steps need not be exactly25ms or share its phase.
  advance(opening + 4999932);
  advance(opening + 24999932);
  ASSERT_EQ(calls_.size(), 1U);
  advance(opening + 29999931);
  ASSERT_EQ(calls_.size(), 2U);
  EXPECT_EQ(calls_[1] - calls_[0], 25000000);
  advance(opening + 54999930);
  ASSERT_EQ(calls_.size(), 3U);
  EXPECT_EQ(calls_[2] - calls_[1], 25000000);
}

TEST_F(PublicationAppointment, LateWakeDoesNotMoveTheAppointmentOrGrantAGraceWindow) {
  constexpr std::int64_t opening = 100000000, deadline = opening + 25000000;
  advance(opening - 10000000);
  alarm_->arm(opening);
  advance(deadline + 1);
  ASSERT_EQ(calls_.size(), 1U);
  EXPECT_EQ(calls_.front(), opening);
  EXPECT_GT(actual_.front(), deadline); // A final window guard must still reject.
  advance(deadline + 25000000);
  EXPECT_EQ(calls_.size(), 1U);
}

TEST_F(PublicationAppointment, AlreadyLateArmRemainsAnImmediateOriginalAppointment) {
  advance(200000000);
  alarm_->arm(100000000);
  advance(200000000);
  ASSERT_EQ(calls_.size(), 1U);
  EXPECT_EQ(calls_.front(), 100000000);
  EXPECT_EQ(actual_.front(), 200000000);
}

TEST_F(PublicationAppointment, RearmingCancelsAnAlreadyReadyOlderTimer) {
  advance(100000000);
  alarm_->arm(100000000);
  alarm_->arm(200000000);
  advance(100000000);
  EXPECT_TRUE(calls_.empty());
  advance(200000000);
  ASSERT_EQ(calls_.size(), 1U);
  EXPECT_EQ(calls_.front(), 200000000);
}

TEST_F(PublicationAppointment, CancelPreventsFurtherCallbacks) {
  alarm_->arm(100000000);
  alarm_->cancel();
  advance(200000000);
  EXPECT_TRUE(calls_.empty());
}

TEST_F(PublicationAppointment, BackwardClockJumpCannotRephaseAnAlarmBeforeItsOriginalOpening) {
  advance(80000000);
  alarm_->arm(100000000);
  advance(40000000);
  advance(60000000);
  advance(99999999);
  EXPECT_TRUE(calls_.empty());
  advance(100000000);
  ASSERT_EQ(calls_.size(), 1U);
  EXPECT_EQ(calls_.front(), 100000000);
}

TEST_F(PublicationAppointment, InvalidAppointmentCancelsTheOldPendingWake) {
  alarm_->arm(100000000);
  EXPECT_THROW(alarm_->arm(-1), std::invalid_argument);
  advance(200000000);
  EXPECT_TRUE(calls_.empty());
}
using Watchdog = multi_purpose_mpc_ros::mpcc_publication_appointment::ClockWatchdog;
using ClockStatus = multi_purpose_mpc_ros::mpcc_publication_appointment::ClockStatus;

TEST(PublicationClockWatchdog, EqualClockSamplesCannotRenewTheExistingSafetyTimeout) {
  using namespace std::chrono_literals;
  const Watchdog::SteadyClock::time_point start{};
  Watchdog watchdog(100, start);
  for (int tick = 1; tick <= 20; ++tick)
    EXPECT_EQ(watchdog.observe(100, start + tick * 25ms, 500ms), ClockStatus::Current);
  EXPECT_EQ(watchdog.observe(100, start + 500ms + 1ns, 500ms), ClockStatus::Stalled);
  EXPECT_EQ(watchdog.observe(100, start + 525ms, 500ms), ClockStatus::Stalled);
}

TEST(PublicationClockWatchdog, ActualClockProgressRestartsTheSteadySafetyInterval) {
  using namespace std::chrono_literals;
  const Watchdog::SteadyClock::time_point start{};
  Watchdog watchdog(100, start);
  EXPECT_EQ(watchdog.observe(100, start + 600ms, 500ms), ClockStatus::Stalled);
  EXPECT_EQ(watchdog.observe(101, start + 625ms, 500ms), ClockStatus::Current);
  EXPECT_EQ(watchdog.observe(101, start + 1125ms, 500ms), ClockStatus::Current);
  EXPECT_EQ(watchdog.observe(101, start + 1125ms + 1ns, 500ms), ClockStatus::Stalled);
}

TEST(PublicationClockWatchdog, ClockRegressionIsImmediateAndIndependentOfTheStallTimeout) {
  using namespace std::chrono_literals;
  const Watchdog::SteadyClock::time_point start{};
  Watchdog watchdog(100, start);
  EXPECT_EQ(watchdog.observe(99, start + 25ms, 500ms), ClockStatus::Regressed);
  EXPECT_EQ(watchdog.observe(99, start + 50ms, 500ms), ClockStatus::Current);
  EXPECT_EQ(watchdog.observe(99, start + 526ms, 500ms), ClockStatus::Stalled);
}
} // namespace
