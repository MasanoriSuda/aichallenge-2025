#pragma once

#include "multi_purpose_mpc_ros/mpcc_applied_input_prediction.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"

namespace multi_purpose_mpc_ros::mpcc_stop_input_program {
namespace vehicle = mpcc_vehicle_model;
namespace artifact = mpcc_rate_resolved_execution_artifact;
using Sample = mpcc_rate_resolved_physical_adapter::PhysicalActuationSample;

/// This is candidate generation, never a physical certificate. The nominal
/// nine-state reference keeps its own control-origin clock. This explicitly
/// sampled wire program begins at publication, with one common sequence for
/// every possible receiver response. It must be independently proved to rest.
struct Request {
  artifact::Identity source;
  std::uint64_t decision_id{};
  double nominal_control_origin_sec{};
  double publication_interval_sec{};
  vehicle::PublishedCommand first_packet;
  double steering_wire_gain{};
  double minimum_acceleration_mps2{};
  double maximum_acceleration_mps2{};
  double maximum_abs_steering_rad{};
  double maximum_abs_steering_rate_radps{};
  /// Existing numerical actuator tolerance, not a new physical margin.
  double actuator_tolerance{};
  std::vector<Sample> nominal_stop_samples;
  double maximum_publication_delay_sec{};
  std::optional<vehicle::PublicationNanosecondClock> nanosecond_clock{};
};

struct Prepared {
  artifact::Identity source;
  std::uint64_t decision_id{};
  double nominal_control_origin_sec{};
  vehicle::PublishedInputProgram program;
  /// A continuous reference sampled at publication boundaries is a different
  /// physical input law. Do not label its old point trajectory as executed.
  bool resampled_controls{false};
};

enum class Reason {
  Available,
  InvalidIdentity,
  InvalidTiming,
  InvalidLimits,
  InvalidSamples,
  NominalRestUnavailable,
  FirstPacketMismatch,
  AccelerationOutsideBounds,
  SteeringOutsideBounds,
  SteeringStepOutsideBounds,
};

struct Result {
  Reason reason{Reason::InvalidIdentity};
  std::optional<Prepared> prepared;
};

Result prepare(const Request &request) noexcept;

/// Resolve the same common program on its publication clock. A new decision
/// must re-prove the returned packet and remaining sequence in its new world.
/// This utility grants neither execution authority nor a duration extension.
std::optional<vehicle::PublishedInputProgram>
remaining_program(const vehicle::PublishedInputProgram &source,
                  double publication_sec) noexcept;

} // namespace multi_purpose_mpc_ros::mpcc_stop_input_program
