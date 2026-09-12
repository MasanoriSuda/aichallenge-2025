#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot
{

/// Diagnostic copy of already received values. Steady timestamps belong to
/// this process; source timestamps belong to ROS. Neither is a bag receipt.
struct ReceivedBodySample
{
  double source_sec{};
  std::int64_t received_steady_ns{};
  double first{};
  double second{};
};

/// Optional evidence only: never part of a model, problem or certificate.
/// Shared const ownership keeps the source queues distinct from later queues.
struct ReceivedBodyObservation
{
  std::uint64_t control_decision_id{};
  std::int64_t captured_steady_ns{};
  double now_sec{};
  double pose_source_sec{};
  std::array<double, 3> selected_component_source_sec{};  // velocity, IMU, tire
  std::array<double, 8> selected_x_y_yaw_u_vy_r_desired_tire{};
  std::array<std::vector<ReceivedBodySample>, 3> histories;
};

}  // namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot
