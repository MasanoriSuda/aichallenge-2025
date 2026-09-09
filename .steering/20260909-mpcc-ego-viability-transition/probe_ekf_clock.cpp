// Diagnostic only: deterministic clock reads for the INSTALLED EKF binary.
// Never loaded by the simulator or a production ROS process.
#include <ekf_localizer/ekf_localizer.hpp>
#include <ekf_localizer/state_index.hpp>
#include <dlfcn.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
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
  static bool run(EKFLocalizer & node, const char * name, std::vector<std::int64_t> times)
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
    clock_values = std::move(times); clock_reads = 0;
    injected_clock = node.get_clock().get();
    node.timerCallback();
    injected_clock = nullptr;
    const double state_epoch = previous.seconds() + node.ekf_dt_;
    const double stored_epoch = node.last_predict_time_->seconds();
    const double pose_epoch = rclcpp::Time(node.current_ekf_pose_.header.stamp).seconds();
    const double twist_epoch = rclcpp::Time(node.current_ekf_twist_.header.stamp).seconds();
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
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EKFLocalizer>("ekf_clock_probe", rclcpp::NodeOptions());
  int failed = 0;
  failed += !EKFLocalizerTestSuite::run(*node, "fixed-clock", {10000000000LL});
  failed += !EKFLocalizerTestSuite::run(*node, "clock-advances-before-storing-epoch",
    {10000000000LL,10000000000LL,10005000000LL});
  failed += !EKFLocalizerTestSuite::run(*node, "clock-advances-before-publishing-result",
    {10000000000LL,10000000000LL,10000000000LL,10005000000LL});
  node.reset(); rclcpp::shutdown();
  return failed == 0 ? 0 : 1;
}
