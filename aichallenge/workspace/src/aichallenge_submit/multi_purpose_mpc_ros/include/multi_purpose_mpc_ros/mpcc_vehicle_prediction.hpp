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

/// Record the serialized packet at its observed publication epoch. The nominal
/// decision epoch is a causal lower bound when the ROS clock cache trails it.
/// Invalid inputs leave history unchanged; repeated epochs retain the last value.
bool record_serialized_publication(
  std::vector<PublishedCommand> & history, const PublishedCommand & nominal_packet,
  double publication_clock_sec, double retain_sec) noexcept;

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

/// Proposed serialized packet, distinct from every already published input.
/// The epoch is the nominal publication time used by this candidate's proof.
struct ProspectivePublicationPrediction
{
  ObservationProvenance observation;
  PublishedCommand proposed_packet;
  State current;
  State control_origin;
  std::vector<TimedState> current_to_control;
  std::uint64_t vehicle_model_fingerprint{};
};

bool publication_packet_matches(
  const ProspectivePublicationPrediction & prediction, double publication_sec,
  double wire_acceleration_mps2, double physical_steering_rad,
  double steering_wire_gain) noexcept;

/// Bind the observation-to-control prefix to the packet being considered now.
/// Historical provenance is retained unchanged; it never claims the proposed
/// packet was already sent. Channel delays remain nominal model assumptions.
std::optional<ProspectivePublicationPrediction> predict_prospective_publication(
  const ObservationProvenance & observation, const PublishedCommand & proposed_packet,
  const Parameters & parameters) noexcept;

/// Piecewise held, already serialized inputs. Channel delays are nominal
/// scheduling assumptions, not acknowledgements or transport guarantees.
/// The tire dynamics begin after the steering channel's delay exactly once.
/// No observed acceleration residual or future body/contact input is added.
std::optional<PublishedPrediction> predict_published_history(
  const TimedState & initial, double now_sec, double control_origin_sec,
  const std::vector<PublishedCommand> & commands, const Parameters & parameters,
  double acceleration_delay_sec, double steering_delay_sec) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_vehicle_model
