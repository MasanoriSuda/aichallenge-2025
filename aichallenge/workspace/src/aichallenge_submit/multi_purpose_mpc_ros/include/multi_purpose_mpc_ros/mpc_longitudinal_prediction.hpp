#pragma once

#include "multi_purpose_mpc_ros/mpc_state_prediction.hpp"

#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::mpc_state_prediction
{

struct ControlObservationEpoch
{
  bool valid{false};
  bool clock_regressed{false};
  bool use_received_observation_stamp{false};
};

/// ROS clock and odometry subscriptions are independently delivered. A state
/// already received by this callback may have a newer source time than the
/// local clock cache. Own the decision at the later causal event, retaining
/// the original integer ROS stamp. Clock reset and out-of-contract skew stay
/// invalid; this never admits a prediction at an earlier observation epoch.
ControlObservationEpoch resolve_control_observation_epoch(
  double ros_clock_sec, std::optional<double> previous_ros_clock_sec,
  double received_observation_sec, double maximum_observation_age_sec) noexcept;

struct AccelerationInterval
{
  double duration_sec{};
  double acceleration_mps2{};
};

struct LongitudinalResponseObservation
{
  double observed_sec{};
  double speed_mps{};
  double filtered_measured_acceleration_mps2{};
  double filtered_command_acceleration_mps2{};

  double response_residual_mps2() const noexcept
  {
    return filtered_measured_acceleration_mps2 -
           filtered_command_acceleration_mps2;
  }
};

/// Pair the measured velocity derivative and the committed input through the
/// same observation filter. Their difference estimates the unmodeled response
/// (drag, grip, etc.). Filtering only the measured side would interpret a
/// committed brake transition as a persistent positive acceleration.
/// This observer owns no command or normal-control authority.
class LongitudinalResponseObserver
{
public:
  LongitudinalResponseObserver(double filter_gain, double maximum_observation_gap_sec);

  /// Record only after successful final serialization/publication. Equal
  /// publication stamps replace the latest queued value. A backward publisher
  /// clock starts a new history; future commands are never predicted inputs.
  bool record_published_command(
    double published_sec, double control_origin_sec, double acceleration_mps2);

  /// Both filters advance once per positive source-time interval. Duplicate
  /// stamps refresh the velocity anchor without inventing elapsed time.
  /// First observations, gaps and backward time require a second observation.
  std::optional<LongitudinalResponseObservation> observe(
    double source_sec, double speed_mps);

  /// Integrate already published controls at their declared origins, adding
  /// the observed response residual. Uses the existing prediction duration;
  /// this is not an identification of simulator command application phase.
  std::optional<std::vector<AccelerationInterval>> prediction_intervals(
    double now_sec, double duration_sec) const;

  void reset() noexcept;
  std::size_t retained_command_count() const noexcept {return commands_.size();}

private:
  struct Command
  {
    double published_sec{};
    double control_origin_sec{};
    double acceleration_mps2{};
  };

  double command_at(double time_sec) const noexcept;
  std::vector<AccelerationInterval> command_intervals(double begin_sec, double end_sec) const;
  double filter_gain_{};
  double maximum_observation_gap_sec_{};
  std::vector<Command> commands_;
  std::optional<double> previous_observation_sec_;
  double previous_speed_mps_{};
  std::optional<LongitudinalResponseObservation> observation_;
};

/// One pose/speed/yaw-response rollout, with exact acceleration switch times.
/// The existing physical-steering interpolation spans the complete duration.
std::vector<TimedYawResponsePrediction> predict_piecewise_yaw_response_trajectory(
  State2D state, double speed_mps, double response_steering_rad,
  double initial_physical_steering_rad, double terminal_physical_steering_rad,
  double wheelbase_m, double yaw_response_gain, double yaw_response_time_constant_sec,
  const std::vector<AccelerationInterval> & intervals);

}  // namespace multi_purpose_mpc_ros::mpc_state_prediction
