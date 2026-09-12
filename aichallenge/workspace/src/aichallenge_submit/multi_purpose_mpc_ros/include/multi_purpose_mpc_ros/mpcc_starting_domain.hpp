#pragma once

#include "multi_purpose_mpc_ros/mpcc_applied_input_prediction.hpp"

namespace multi_purpose_mpc_ros::mpcc_vehicle_model {

/// Numerical data only. A short prefix is not a complete-rest certificate.
struct CurrentInputPrefix {
  std::uint64_t context_fingerprint{};
  ObservationProvenance observation;
  PublishedInputProgram program;
  InputApplicationProfile profile;
  State coordinate_origin;
  FootprintRanges footprint_offsets;
  BodyRanges body;
  FootprintRanges footprint;
};
struct CurrentInputPrefixPrediction {
  AppliedInputRejectReason reason{AppliedInputRejectReason::InvalidObservation};
  std::optional<CurrentInputPrefix> prefix;
};

/// Same original observation/history and variable native prefix grid as the
/// complete pending predictor, stopping exactly at observation.now_sec.
CurrentInputPrefixPrediction
predict_pending_input_prefix(const ObservationProvenance &observation,
                             const PublishedInputProgram &program,
                             const InputApplicationProfile &profile,
                             const Parameters &parameters,
                             const FootprintRanges &footprint_offsets) noexcept;

/// A candidate set of possible starting states, not a sensor error bound.
/// Every start time is inside the first original unsent packet's window.
/// Body XY/yaw use coordinate_origin's fixed heading; corners use world axes
/// relative to its XY. No old future tube is translated or reused as this
/// proof.
struct StartingDomainRequest {
  ObservationProvenance source_observation;
  PublishedInputProgram program;
  InputApplicationProfile profile;
  State coordinate_origin;
  BodyRanges body;
  FootprintRanges footprint_offsets;
  ScalarRange starting_sec;
};
struct StartingDomainSample {
  double relative_begin_sec{};
  double relative_end_sec{};
  double absolute_begin_sec{};
  double absolute_end_sec{};
  AppliedInputBounds inputs;
  BodyRanges swept_body;
  BodyRanges endpoint_body;
  FootprintRanges swept_footprint;
  FootprintRanges endpoint_footprint;
};
struct StartingDomainTube {
  std::uint64_t context_fingerprint{};
  StartingDomainRequest request;
  FootprintRanges initial_footprint;
  std::vector<StartingDomainSample> source_to_rest;
  double rest_sec{};
  std::size_t maximum_body_partitions{};
};
struct StartingDomainPrediction {
  AppliedInputRejectReason reason{AppliedInputRejectReason::InvalidObservation};
  std::optional<StartingDomainTube> tube;
};

std::uint64_t
starting_domain_context_fingerprint(const StartingDomainRequest &request,
                                    const Parameters &parameters) noexcept;

/// Freshly propagate the entire starting box at every allowed starting time.
/// Future native steps retain maximum_step_sec; input selection includes the
/// whole absolute time interval. Returns no partial result without full rest.
/// Numerical evidence only: wall/peer/Follow, original source/context/ledger,
/// current whole-prefix membership and actual final guards remain necessary.
StartingDomainPrediction
predict_starting_domain_to_rest(const StartingDomainRequest &request,
                                const Parameters &parameters) noexcept;

/// Separate numerical theorem for a complete original programme, including
/// its repeated braking tail. The nested starting set/time is data; it is not
/// validated as a first-publication-window request. No send window is widened.
/// Original history/programme retain already sent packets' delayed input memory.
struct ProgrammeStartingDomainRequest {
  StartingDomainRequest domain;
  double original_rest_sec{};
};
struct ProgrammeStartingDomainTube {
  StartingDomainTube numerical;
  double original_rest_sec{};
};
struct ProgrammeStartingDomainPrediction {
  AppliedInputRejectReason reason{AppliedInputRejectReason::InvalidObservation};
  std::optional<ProgrammeStartingDomainTube> tube;
};
std::uint64_t programme_starting_domain_context_fingerprint(
    const ProgrammeStartingDomainRequest &request,
    const Parameters &parameters) noexcept;
ProgrammeStartingDomainPrediction predict_programme_starting_domain_to_rest(
    const ProgrammeStartingDomainRequest &request,
    const Parameters &parameters) noexcept;

/// A distinct complete numerical theorem: body/time population is independently
/// propagated from zero XY/yaw. Its future is composed with a whole current pose,
/// never with an old point trajectory. Original programme/time/input memory stays.
struct RelativeProgrammeStartingDomainTube {
  ProgrammeStartingDomainTube normalized;
};
struct RelativeProgrammeStartingDomainPrediction {
  AppliedInputRejectReason reason{AppliedInputRejectReason::InvalidObservation};
  std::optional<RelativeProgrammeStartingDomainTube> tube;
};
RelativeProgrammeStartingDomainPrediction predict_relative_programme_domain_to_rest(
    const ProgrammeStartingDomainRequest &request,
    const Parameters &parameters) noexcept;

/// Numerical composition only. Construction requires full body/time membership
/// and exact footprint offsets. Source/history/world/authority checks remain in
/// the dispatcher. Ranges and swept corners are composed with outward arithmetic.
class RelativeDomainTransform {
public:
  static std::optional<RelativeDomainTransform> build(
    const RelativeProgrammeStartingDomainTube &domain, const CurrentInputPrefix &prefix) noexcept;
  BodyRanges body(const BodyRanges &relative) const;
  FootprintRanges footprint(const FootprintRanges &relative) const;
private:
  RelativeDomainTransform() = default;
  std::array<ScalarRange, 9> coefficients_{};
};

/// Strict geometric/state/time membership only, including every prefix range.
/// This neither authenticates original history nor grants command authority.
/// Exact original footprint offsets must agree; frame rotations are outward.
bool starting_domain_contains_prefix(const StartingDomainRequest &domain,
                                     const CurrentInputPrefix &prefix) noexcept;

} // namespace multi_purpose_mpc_ros::mpcc_vehicle_model
