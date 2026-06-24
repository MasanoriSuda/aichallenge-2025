#ifndef MULTI_PURPOSE_MPC_ROS_CPP__REF_VELOCITY_CONFIG_HPP_
#define MULTI_PURPOSE_MPC_ROS_CPP__REF_VELOCITY_CONFIG_HPP_

#include <string>
#include <utility>
#include <vector>

namespace multi_purpose_mpc_ros_cpp
{

class RefVelocityConfig
{
public:
  explicit RefVelocityConfig(const std::string & path);

  bool empty() const;
  double get_ref_vel_kmph(std::size_t current_wp_id) const;

private:
  std::vector<std::pair<std::size_t, double>> sections_;
};

}  // namespace multi_purpose_mpc_ros_cpp

#endif  // MULTI_PURPOSE_MPC_ROS_CPP__REF_VELOCITY_CONFIG_HPP_
