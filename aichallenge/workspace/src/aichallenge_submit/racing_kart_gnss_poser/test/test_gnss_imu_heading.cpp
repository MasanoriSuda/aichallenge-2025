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
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
constexpr double kPi = 3.14159265358979323846;

class ImuHeadingTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

class Harness
{
public:
  explicit Harness(const std::string & name, const bool extrinsics = true)
  : local_(35.626, 139.781, 42.5), antenna_(name + "_antenna"), imu_frame_(name + "_imu")
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/" + name});
    options.parameter_overrides({
      rclcpp::Parameter("base_frame", name + "_base"),
      rclcpp::Parameter("gnss_frame", antenna_),
      rclcpp::Parameter("gnss_base_frame", name + "_result"),
      rclcpp::Parameter("coordinate_system", 3),
      rclcpp::Parameter("latitude", 35.626), rclcpp::Parameter("longitude", 139.781),
      rclcpp::Parameter("altitude", 42.5),
      rclcpp::Parameter("use_gnss_ins_orientation", false),
      rclcpp::Parameter("use_imu_orientation", true),
      rclcpp::Parameter("gnss_change_threshold", 0.2)});
    poser_ = std::make_shared<gnss_poser::GNSSPoser>(options);
    peer_ = std::make_shared<rclcpp::Node>("imu_heading_peer", "/" + name);
    subscription_ = peer_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "gnss_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        received = msg; ++count;
      });
    fixed_subscription_ = peer_->create_subscription<tier4_debug_msgs::msg::BoolStamped>(
      "gnss_fixed", 10, [this](tier4_debug_msgs::msg::BoolStamped::SharedPtr msg) {
        last_fixed_status = msg->data; ++fixed_count;
      });
    fixes_ = peer_->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
    imus_ = peer_->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    executor_.add_node(poser_);
    executor_.add_node(peer_);
    if (extrinsics) {
      broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(peer_);
      geometry_msgs::msg::TransformStamped gnss, imu_transform;
      gnss.header.frame_id = imu_transform.header.frame_id = name + "_base";
      gnss.child_frame_id = antenna_;
      gnss.transform.translation.x = -0.26;
      gnss.transform.rotation.w = 1.0;
      imu_transform.child_frame_id = imu_frame_;
      imu_transform.transform.translation.x = 0.85;
      imu_transform.transform.rotation.z = std::sin(kPi / 4.0);
      imu_transform.transform.rotation.w = std::cos(kPi / 4.0);
      broadcaster_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{gnss, imu_transform});
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while ((fixes_->get_subscription_count() == 0 || imus_->get_subscription_count() == 0) &&
      std::chrono::steady_clock::now() < deadline)
    {
      pump();
    }
    if (!fixes_->get_subscription_count() || !imus_->get_subscription_count()) {
      throw std::runtime_error("IMU heading discovery failed");
    }
    pump();
  }

  sensor_msgs::msg::NavSatFix fix(const int second = 10) const
  {
    sensor_msgs::msg::NavSatFix result;
    result.header.frame_id = antenna_;
    result.header.stamp.sec = second;
    result.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    local_.Reverse(10.0, 20.0, 0.0, result.latitude, result.longitude, result.altitude);
    return result;
  }

  sensor_msgs::msg::Imu imu(const int second = 10, const double body_yaw = 2.035) const
  {
    sensor_msgs::msg::Imu result;
    result.header.frame_id = imu_frame_;
    result.header.stamp.sec = second;
    result.orientation.z = std::sin((body_yaw + kPi / 2.0) / 2.0);
    result.orientation.w = std::cos((body_yaw + kPi / 2.0) / 2.0);
    return result;
  }

  void send(const sensor_msgs::msg::NavSatFix & fix) {fixes_->publish(fix); pump();}
  void send(const sensor_msgs::msg::Imu & imu) {imus_->publish(imu); pump();}
  void pump()
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(60);
    do {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (std::chrono::steady_clock::now() < end);
  }

  geometry_msgs::msg::PoseStamped::SharedPtr received;
  unsigned count{};
  unsigned fixed_count{};
  bool last_fixed_status{};

private:
  GeographicLib::LocalCartesian local_;
  std::string antenna_, imu_frame_;
  std::shared_ptr<gnss_poser::GNSSPoser> poser_;
  std::shared_ptr<rclcpp::Node> peer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Subscription<tier4_debug_msgs::msg::BoolStamped>::SharedPtr fixed_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fixes_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imus_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(ImuHeadingTest, TransformsAbsoluteAttitudeBeforeLeverArmAtRest)
{
  Harness h("imu_lever");
  const auto imu = h.imu();
  const auto fix = h.fix();
  h.send(imu);
  h.send(fix);
  ASSERT_TRUE(h.received);
  const auto & q = h.received->pose.orientation;
  EXPECT_NEAR(std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)), 2.035, 1e-12);
  EXPECT_NEAR(h.received->pose.position.x, 10.0+0.26*std::cos(2.035), 1e-5);
  EXPECT_NEAR(h.received->pose.position.y, 20.0+0.26*std::sin(2.035), 1e-5);
  EXPECT_EQ(h.received->header.stamp, fix.header.stamp);
  h.send(fix);
  h.send(imu);
  EXPECT_EQ(h.count, 1U);
}

TEST_F(ImuHeadingTest, FixFirstWaitsForExactEpochAndDoesNotUseOlderAttitude)
{
  Harness h("imu_epoch");
  h.send(h.imu(9));
  h.send(h.fix(10));
  EXPECT_EQ(h.count, 0U);
  h.send(h.imu(10));
  ASSERT_EQ(h.count, 1U);
  h.send(h.imu(12));
  h.send(h.fix(11));
  EXPECT_EQ(h.count, 1U);
  h.send(h.fix(12));
  EXPECT_EQ(h.count, 2U);
}

TEST_F(ImuHeadingTest, InvalidOrUnavailableAttitudeAndWrongFixFrameDoNotPublish)
{
  Harness h("imu_invalid");
  auto imu = h.imu(10);
  imu.orientation.z = imu.orientation.w = 0.0;
  h.send(imu); h.send(h.fix(10));
  EXPECT_EQ(h.count, 0U);
  imu = h.imu(11); imu.orientation.x = std::numeric_limits<double>::quiet_NaN();
  h.send(imu); h.send(h.fix(11));
  EXPECT_EQ(h.count, 0U);
  imu = h.imu(12); imu.orientation_covariance[0] = -1.0;
  h.send(imu); h.send(h.fix(12));
  EXPECT_EQ(h.count, 0U);
  h.send(h.imu(13));
  auto fix = h.fix(13); fix.header.frame_id = "wrong_antenna";
  h.send(fix);
  EXPECT_EQ(h.count, 0U);
  h.send(h.imu(14)); h.send(h.fix(14));
  EXPECT_EQ(h.count, 1U);
}

TEST_F(ImuHeadingTest, MissingExtrinsicsDoesNotUseIdentityTransform)
{
  Harness h("imu_no_tf", false);
  h.send(h.imu()); h.send(h.fix());
  EXPECT_EQ(h.count, 0U);
}

TEST_F(ImuHeadingTest, ReportsReceiverFixStatusWithoutImuAttitude)
{
  Harness h("imu_fix_status");
  h.send(h.fix());
  EXPECT_EQ(h.count, 0U);
  EXPECT_EQ(h.fixed_count, 1U);
  EXPECT_TRUE(h.last_fixed_status);
  auto lost = h.fix(11);
  lost.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  h.send(lost);
  EXPECT_EQ(h.fixed_count, 2U);
  EXPECT_FALSE(h.last_fixed_status);
  h.send(h.imu(10));
  EXPECT_EQ(h.count, 0U);
}

TEST_F(ImuHeadingTest, ConflictingOrientationAndMultiEpochMedianAreRejected)
{
  for (const bool ins : {false, true}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_imu_orientation", true),
      rclcpp::Parameter("use_gnss_ins_orientation", ins),
      rclcpp::Parameter("buff_epoch", ins ? 1 : 2),
      rclcpp::Parameter("gnss_change_threshold", 0.2)});
    EXPECT_THROW(std::make_shared<gnss_poser::GNSSPoser>(options), std::invalid_argument);
  }
}
}  // namespace
