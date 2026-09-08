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

#include <chrono>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

namespace
{
class GNSSCovarianceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  rclcpp::NodeOptions options(const std::vector<rclcpp::Parameter> & extra = {})
  {
    std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter("base_frame", "base_link"),
      rclcpp::Parameter("gnss_frame", "base_link"),
      rclcpp::Parameter("coordinate_system", 3),
      rclcpp::Parameter("latitude", 35.626),
      rclcpp::Parameter("longitude", 139.781),
      rclcpp::Parameter("altitude", 42.5),
      rclcpp::Parameter("use_gnss_ins_orientation", false),
      rclcpp::Parameter("gnss_change_threshold", 0.2)};
    parameters.insert(parameters.end(), extra.begin(), extra.end());
    return rclcpp::NodeOptions().parameter_overrides(parameters);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr publish_fix(
    const rclcpp::NodeOptions & node_options, const sensor_msgs::msg::NavSatFix & fix)
  {
    auto poser = std::make_shared<gnss_poser::GNSSPoser>(node_options);
    auto peer = std::make_shared<rclcpp::Node>("covariance_test_peer");
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr received;
    auto subscription = peer->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/gnss_pose_cov", 10, [&received](
        geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {received = message;});
    auto publisher = peer->create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 10);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(poser);
    executor.add_node(peer);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!received && std::chrono::steady_clock::now() < deadline) {
      publisher->publish(fix);
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return received;
  }

  sensor_msgs::msg::NavSatFix measurement()
  {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.frame_id = "base_link";
    fix.header.stamp.sec = 250;
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.latitude = 35.626;
    fix.longitude = 139.781;
    fix.altitude = 42.5;
    return fix;
  }
};

TEST_F(GNSSCovarianceTest, UnknownUsesExplicitSensorCalibration)
{
  const auto result = publish_fix(
    options({rclcpp::Parameter("unknown_position_covariance", 0.1)}), measurement());
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->header.stamp.sec, 250);
  for (const auto index : {0, 7, 14}) {
    EXPECT_DOUBLE_EQ(result->pose.covariance[index], 0.1);
  }
}

TEST_F(GNSSCovarianceTest, UncalibratedUnknownPreservesExistingDefault)
{
  const auto result = publish_fix(options(), measurement());
  ASSERT_NE(result, nullptr);
  for (const auto index : {0, 7, 14}) {
    EXPECT_DOUBLE_EQ(result->pose.covariance[index], 10.0);
  }
}

TEST_F(GNSSCovarianceTest, KnownDiagonalIsNotReplacedByFallback)
{
  auto fix = measurement();
  fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix.position_covariance[0] = 0.02;
  fix.position_covariance[4] = 0.03;
  fix.position_covariance[8] = 0.04;
  const auto result = publish_fix(
    options({rclcpp::Parameter("unknown_position_covariance", 0.1)}), fix);
  ASSERT_NE(result, nullptr);
  EXPECT_DOUBLE_EQ(result->pose.covariance[0], 0.02);
  EXPECT_DOUBLE_EQ(result->pose.covariance[7], 0.03);
  EXPECT_DOUBLE_EQ(result->pose.covariance[14], 0.04);
}

TEST_F(GNSSCovarianceTest, InvalidCalibrationCannotStart)
{
  for (const double covariance : {
      0.0, -1.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    EXPECT_THROW(
      std::make_shared<gnss_poser::GNSSPoser>(
        options({rclcpp::Parameter("unknown_position_covariance", covariance)})),
      std::invalid_argument);
  }
}
}  // namespace
