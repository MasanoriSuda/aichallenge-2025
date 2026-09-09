// Copyright 2018-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define _USE_MATH_DEFINES
// Clock interposition exists only in this isolated test executable.
#include <aichallenge_ekf_localizer/ekf_localizer.hpp>
#include <aichallenge_ekf_localizer/state_index.hpp>
#include <dlfcn.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <optional>
#include <gtest/gtest.h>
namespace {
thread_local rclcpp::Clock * injected_clock = nullptr;
thread_local std::vector<std::int64_t> clock_values;
thread_local std::size_t clock_reads = 0;
}
rclcpp::Time rclcpp::Clock::now()
{
  if (this == injected_clock) {
    const auto i = std::min(clock_reads++, clock_values.size()-1U);
    return rclcpp::Time(clock_values[i], RCL_ROS_TIME);
  }
  using Original = rclcpp::Time (*)(rclcpp::Clock *);
  static auto original = reinterpret_cast<Original>(dlsym(RTLD_NEXT, "_ZN6rclcpp5Clock3nowEv"));
  if (!original) throw std::runtime_error("cannot resolve installed Clock::now");
  return original(this);
}
class EKFLocalizerTestSuite
{
public:
  static bool run(EKFLocalizer & node, const char * name, std::vector<std::int64_t> times, bool with_measurements = false)
  {
    node.timer_control_->cancel(); node.timer_tf_->cancel();
    const rclcpp::Time previous(9980000000LL, RCL_ROS_TIME);
    Eigen::MatrixXd x = Eigen::MatrixXd::Zero(6, 1);
    x(IDX::VX) = 1.0;
    const Eigen::MatrixXd p = Eigen::MatrixXd::Identity(6, 6)*0.01;
    node.ekf_.init(x, p, node.params_.extend_state_step);
    node.z_filter_.init(0.0, 0.01, previous);
    node.roll_filter_.init(0.0, 0.01, previous);
    node.pitch_filter_.init(0.0, 0.01, previous);
    node.last_predict_time_ = std::make_shared<const rclcpp::Time>(previous);
    node.is_activated_ = true;
    if (with_measurements) {
      auto pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
      pose->header.frame_id = "map";
      pose->header.stamp = rclcpp::Time(9990000000LL, RCL_ROS_TIME);
      pose->pose.pose.position.x = 0.01;
      pose->pose.pose.orientation.w = 1.0;
      for (int i = 0; i < 6; ++i) pose->pose.covariance[i*7] = 0.01;
      auto twist = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
      twist->header.frame_id = "base_link";
      twist->header.stamp = pose->header.stamp;
      twist->twist.twist.linear.x = 1.0;
      for (int i = 0; i < 6; ++i) twist->twist.covariance[i*7] = 0.01;
      node.callbackPoseWithCovariance(pose);
      node.callbackTwistWithCovariance(twist);
    }
    std::optional<nav_msgs::msg::Odometry> published;
    auto subscription = node.create_subscription<nav_msgs::msg::Odometry>(
      node.pub_odom_->get_topic_name(), rclcpp::QoS(1),
      [&published](nav_msgs::msg::Odometry::ConstSharedPtr message) {published = *message;});
    clock_values = std::move(times); clock_reads = 0;
    injected_clock = node.get_clock().get();
    node.timerCallback();
    injected_clock = nullptr;
    const double state_epoch = previous.seconds() + node.ekf_dt_;
    const double stored_epoch = node.last_predict_time_->seconds();
    const double pose_epoch = rclcpp::Time(node.current_ekf_pose_.header.stamp).seconds();
    const double twist_epoch = rclcpp::Time(node.current_ekf_twist_.header.stamp).seconds();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node.get_node_base_interface());
    executor.spin_some();
    EXPECT_TRUE(published.has_value());
    if (published) {
      EXPECT_DOUBLE_EQ(rclcpp::Time(published->header.stamp).seconds(), state_epoch);
    }
    EXPECT_EQ(clock_reads, 1U);
    const bool invariant = std::abs(state_epoch-stored_epoch)<1e-10 &&
      std::abs(state_epoch-pose_epoch)<1e-10 && std::abs(state_epoch-twist_epoch)<1e-10;
    std::cout.precision(17);
    std::cout << "{\"case\":\"" << name << "\",\"clock_reads\":" << clock_reads
      << ",\"prediction_dt\":" << node.ekf_dt_ << ",\"state_epoch\":" << state_epoch
      << ",\"stored_epoch\":" << stored_epoch << ",\"pose_stamp\":" << pose_epoch
      << ",\"twist_stamp\":" << twist_epoch << ",\"predicted_x\":" << node.ekf_.getXelement(IDX::X)
      << ",\"epoch_invariant\":" << (invariant ? "true" : "false") << "}\n";
    return invariant;
  }
};

class ClockEpochTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<EKFLocalizer>("ekf_clock_test",
      rclcpp::NodeOptions().use_intra_process_comms(true));
  }
  void TearDown() override {node.reset();}
  std::shared_ptr<EKFLocalizer> node;
};
TEST_F(ClockEpochTest, FixedClockUsesActualElapsedTime)
{
  EXPECT_TRUE(EKFLocalizerTestSuite::run(*node, "fixed-clock", {10000000000LL}));
}
TEST_F(ClockEpochTest, ClockAdvanceCannotRelabelStoredPrediction)
{
  EXPECT_TRUE(EKFLocalizerTestSuite::run(*node, "advance-before-storing",
    {10000000000LL, 10000000000LL, 10005000000LL}));
}
TEST_F(ClockEpochTest, ClockAdvanceCannotRelabelPublishedState)
{
  EXPECT_TRUE(EKFLocalizerTestSuite::run(*node, "advance-before-publishing",
    {10000000000LL, 10000000000LL, 10000000000LL, 10005000000LL}));
}
TEST_F(ClockEpochTest, QueuedMeasurementsUseTheSamePredictionEpoch)
{
  EXPECT_TRUE(EKFLocalizerTestSuite::run(*node, "measurements-at-prediction-epoch",
    {10000000000LL, 10005000000LL}, true));
}
