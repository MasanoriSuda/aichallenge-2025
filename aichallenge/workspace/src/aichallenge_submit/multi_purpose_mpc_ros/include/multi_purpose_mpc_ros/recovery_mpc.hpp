#ifndef MULTI_PURPOSE_MPC_ROS__RECOVERY_MPC_HPP_
#define MULTI_PURPOSE_MPC_ROS__RECOVERY_MPC_HPP_

#include <cstddef>
#include <vector>

namespace multi_purpose_mpc_ros::recovery_mpc
{

enum class Direction
{
  Forward,
  Reverse,
};

enum class RejectReason
{
  None,
  InvalidConfig,
  InvalidRequest,
  NoFiniteCandidate,
};

const char * to_string(RejectReason reason) noexcept;

/// Small deterministic finite-horizon controller used only inside bounded Recovery.
///
/// The planner uses signed-distance Frenet bicycle dynamics and a beam search over
/// discrete tire-angle inputs. It is intentionally independent of the normal
/// forward-speed OSQP MPC, whose time/progress model is not valid for reverse motion.
struct Config
{
  std::size_t horizon_steps{8U};
  std::size_t steering_sample_count{9U};
  std::size_t beam_width{32U};
  double travel_step_m{0.20};
  double maximum_steering_angle_rad{0.35};
  double maximum_steering_change_rad{0.12};
  double lateral_error_weight{6.0};
  double heading_error_weight{12.0};
  double steering_weight{0.15};
  double steering_change_weight{1.0};
  double terminal_lateral_error_weight{30.0};
  double terminal_heading_error_weight{40.0};
};

struct Request
{
  Direction direction{Direction::Forward};
  double lateral_error_m{};
  double heading_error_rad{};
  double reference_curvature_radpm{};
  double wheelbase_m{};
  double initial_steering_tire_angle_rad{};
};

struct Result
{
  bool valid{false};
  RejectReason reason{RejectReason::InvalidRequest};
  double cost{};
  double first_steering_tire_angle_rad{};
  double terminal_lateral_error_m{};
  double terminal_heading_error_rad{};
  std::vector<double> steering_sequence_rad;
};

bool config_is_valid(const Config & config) noexcept;

/// Plan one signed-distance recovery horizon and return the first tire angle.
///
/// No state is retained: callers replan on each control cycle. This makes the
/// result deterministic from the current path-relative state and lets the
/// existing Recovery supervisor remain the sole owner of attempt/time bounds.
Result plan(const Config & config, const Request & request);

}  // namespace multi_purpose_mpc_ros::recovery_mpc

#endif  // MULTI_PURPOSE_MPC_ROS__RECOVERY_MPC_HPP_
