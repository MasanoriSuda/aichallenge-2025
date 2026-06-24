#include "multi_purpose_mpc_ros_cpp/controller.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<multi_purpose_mpc_ros_cpp::CppMpcControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
