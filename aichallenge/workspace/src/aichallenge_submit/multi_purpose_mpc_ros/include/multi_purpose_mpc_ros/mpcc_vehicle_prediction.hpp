#pragma once

#include "multi_purpose_mpc_ros/mpcc_vehicle_model.hpp"

#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_vehicle_model
{

struct PublishedCommand
{
  double published_sec{};
  double wire_acceleration_mps2{};
  double wire_steering_rad{};
};

struct TimedState
{
  double source_sec{};
  State state;
};

/// Original public observation and serialized history used to produce an
/// immutable problem. Derived/rebased candidates retain this source evidence.
struct ObservationProvenance
{
  TimedState initial;
  double velocity_source_sec{};
  double yaw_rate_source_sec{};
  double tire_source_sec{};
  double now_sec{};
  double control_origin_sec{};
  double acceleration_delay_sec{};
  double steering_delay_sec{};
  std::vector<PublishedCommand> commands;
};

bool valid(const ObservationProvenance & value) noexcept;

struct PublishedPrediction
{
  ObservationProvenance provenance;
  State current;
  State control_origin;
  std::vector<TimedState> current_to_control;
};

/// Piecewise held, already serialized inputs. Channel delays are nominal
/// scheduling assumptions, not acknowledgements or transport guarantees.
/// The tire dynamics begin after the steering channel's delay exactly once.
/// No observed acceleration residual or future body/contact input is added.
std::optional<PublishedPrediction> predict_published_history(
  const TimedState & initial, double now_sec, double control_origin_sec,
  const std::vector<PublishedCommand> & commands, const Parameters & parameters,
  double acceleration_delay_sec, double steering_delay_sec) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_vehicle_model
