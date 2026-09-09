// Copyright 2026 AI Challenge contributors
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

#include "gnss_poser/gnss_poser_core.hpp"

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

class GNSSHeadingTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

class HeadingHarness
{
public:
  explicit HeadingHarness(const std::string & name, const bool lever_arm = false)
  : local_(35.626, 139.781, 42.5)
  {
    const auto base = name + "_base";
    const auto antenna = lever_arm ? name + "_gnss" : base;
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/" + name});
    options.parameter_overrides({
      rclcpp::Parameter("base_frame", base),
      rclcpp::Parameter("gnss_frame", antenna),
      rclcpp::Parameter("gnss_base_frame", name + "_result"),
      rclcpp::Parameter("coordinate_system", 3),
      rclcpp::Parameter("latitude", 35.626),
      rclcpp::Parameter("longitude", 139.781),
      rclcpp::Parameter("altitude", 42.5),
      rclcpp::Parameter("use_gnss_ins_orientation", false),
      rclcpp::Parameter("gnss_change_threshold", 0.2)});
    poser_ = std::make_shared<gnss_poser::GNSSPoser>(options);
    peer_ = std::make_shared<rclcpp::Node>("heading_peer", "/" + name);
    subscription_ = peer_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "gnss_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {received_ = msg;});
    publisher_ = peer_->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
    gear_ = peer_->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
      "/vehicle/status/gear_status", 10);
    executor_.add_node(poser_);
    executor_.add_node(peer_);
    if (lever_arm) {
      broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(peer_);
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = base;
      transform.child_frame_id = antenna;
      transform.transform.translation.x = -0.26;
      transform.transform.rotation.w = 1.0;
      broadcaster_->sendTransform(transform);
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (publisher_->get_subscription_count() == 0U &&
      std::chrono::steady_clock::now() < deadline)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void reverse()
  {
    autoware_auto_vehicle_msgs::msg::GearReport gear;
    gear.report = autoware_auto_vehicle_msgs::msg::GearReport::REVERSE;
    for (int i = 0; i < 10; ++i) {
      gear_->publish(gear);
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  geometry_msgs::msg::Pose sample(const double x, const double y)
  {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp.sec = 250 + sequence_++;
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    local_.Reverse(x, y, 0.0, fix.latitude, fix.longitude, fix.altitude);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    do {
      publisher_->publish(fix);
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while ((!received_ || received_->header.stamp != fix.header.stamp) &&
      std::chrono::steady_clock::now() < deadline);
    if (!received_ || received_->header.stamp != fix.header.stamp) {
      throw std::runtime_error("heading pose not received");
    }
    return received_->pose;
  }

  static double yaw(const geometry_msgs::msg::Pose & pose)
  {
    const auto & q = pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

private:
  GeographicLib::LocalCartesian local_;
  int sequence_{};
  std::shared_ptr<gnss_poser::GNSSPoser> poser_;
  std::shared_ptr<rclcpp::Node> peer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  geometry_msgs::msg::PoseStamped::SharedPtr received_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(GNSSHeadingTest, SlowNorthAccumulatesBaselineAndRotatesLeverArm)
{
  HeadingHarness harness("north_lever", true);
  geometry_msgs::msg::Pose pose;
  for (int i = 0; i <= 6; ++i) {
    pose = harness.sample(0.0, 0.05 * i);
  }
  EXPECT_NEAR(HeadingHarness::yaw(pose), kPi / 2.0, 1e-5);
  EXPECT_NEAR(pose.position.x, 0.0, 1e-5);
  EXPECT_NEAR(pose.position.y, 0.30 + 0.26, 1e-5);
}

TEST_F(GNSSHeadingTest, SlowWestAccumulatesBaseline)
{
  HeadingHarness harness("west");
  geometry_msgs::msg::Pose pose;
  for (int i = 0; i <= 6; ++i) {
    pose = harness.sample(10.0 - 0.05 * i, 20.0);
  }
  EXPECT_NEAR(std::abs(HeadingHarness::yaw(pose)), kPi, 1e-5);
}

TEST_F(GNSSHeadingTest, ReverseMotionPreservesVehicleFacingDirection)
{
  HeadingHarness harness("reverse_south");
  harness.reverse();
  geometry_msgs::msg::Pose pose;
  for (int i = 0; i <= 6; ++i) {
    pose = harness.sample(0.0, -0.05 * i);
  }
  EXPECT_NEAR(HeadingHarness::yaw(pose), kPi / 2.0, 1e-5);
}

TEST_F(GNSSHeadingTest, IndependentNodesDoNotSharePositionAnchor)
{
  HeadingHarness north("independent_north"), west("independent_west");
  geometry_msgs::msg::Pose n, w;
  for (int i = 0; i <= 6; ++i) {
    n = north.sample(0.0, 0.05 * i);
    w = west.sample(10.0 - 0.05 * i, 20.0);
  }
  EXPECT_NEAR(HeadingHarness::yaw(n), kPi / 2.0, 1e-5);
  EXPECT_NEAR(std::abs(HeadingHarness::yaw(w)), kPi, 1e-5);
}

TEST_F(GNSSHeadingTest, SubthresholdNoiseKeepsAcceptedDirection)
{
  HeadingHarness harness("north_noise");
  harness.sample(0.0, 0.0);
  harness.sample(0.0, 0.30);
  harness.sample(0.02, 0.31);
  const auto pose = harness.sample(-0.02, 0.32);
  EXPECT_NEAR(HeadingHarness::yaw(pose), kPi / 2.0, 1e-5);
}
}  // namespace
