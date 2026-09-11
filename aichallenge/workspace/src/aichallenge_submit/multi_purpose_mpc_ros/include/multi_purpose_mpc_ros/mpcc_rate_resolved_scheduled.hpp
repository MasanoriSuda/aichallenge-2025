#pragma once

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"
#include "multi_purpose_mpc_ros/mpcc_publication_ledger.hpp"

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled {
namespace retained = mpcc_rate_resolved_retained_revalidation;
namespace vehicle = mpcc_vehicle_model;
namespace applied = mpcc_rate_resolved_applied_program;

/// Exact frame used by the captured live progress projection. It must reproduce
/// the original request's progress; a guessed nearest frame is not sufficient.
struct ProgressFrame {
  recovery_footprint::Pose2D pose;
  double progress_m{};
};

struct Request {
  retained::Request observed;
  vehicle::PublishedInputProgram prior_program;
  std::size_t prior_index{};
  std::size_t preceding_packet_count{};
  double planned_control_origin_sec{};
  ProgressFrame progress_frame;
};

struct Result;

/// A solved-source nominal continuation at a future publication epoch. Its
/// original observation remains separate. The private nominal Request/Proof
/// deliberately lack the required ordinary publication/applied certificates,
/// so they cannot execute through the legacy adapters.
class NominalProof {
public:
  const retained::Request &observed() const noexcept { return observed_; }
  const retained::Request &nominal_view() const noexcept { return nominal_view_; }
  const retained::Proof &proof() const noexcept { return proof_; }
  const vehicle::ScheduledPublicationPrediction &forecast() const noexcept { return forecast_; }
  double original_follow_reference_progress_m() const noexcept { return original_follow_reference_progress_m_; }

private:
  NominalProof() = default;
  friend Result evaluate(const Request &);
  retained::Request observed_;
  retained::Request nominal_view_;
  retained::Proof proof_;
  vehicle::ScheduledPublicationPrediction forecast_;
  double original_follow_reference_progress_m_{};
};

struct Result {
  retained::Reason reason{retained::Reason::PublicationPrefixUnavailable};
  applied::ScheduledResult applied;
  /// Diagnostic details only. The ordinary proof pointer is always removed.
  retained::Result diagnostic;
};

/// Build the new packet under the declared old prefix, reselect/bind its nominal
/// continuation, and prove the complete composite programme from original now
/// to rest. Actual prefix, fresh world/sensors and final dispatch remain gates.
Result evaluate(const Request &request);

/// Necessary consistency of fresh public component samples with the ORIGINAL
/// input population. This does not reanchor a new population, authenticate the
/// actual publication prefix or grant authority over a fresh world.
enum class MeasurementReason {
  Compatible,
  InvalidObservation,
  TimeOutsideProof,
  InputDelayMismatch,
  IssuedSteeringMismatch,
  PoseMismatch,
  VelocityMismatch,
  YawRateMismatch,
  TireMismatch,
};
struct MeasurementCheck {
  MeasurementReason reason{MeasurementReason::InvalidObservation};
  double rejected_source_sec{std::numeric_limits<double>::quiet_NaN()};
};
MeasurementCheck check_measurement_consistency(
  const applied::ScheduledCertificate &certificate,
  const vehicle::ObservationProvenance &fresh);

/// Metadata for recording the intended owner of an already selected suffix
/// packet. This grants no publication permission and never changes its epoch.
std::optional<vehicle::PublishedProgramSource> scheduled_program_source(
  const applied::ScheduledCertificate &certificate, std::size_t suffix_index);

enum class PrefixReason {
  Consistent,
  OriginalHistoryMismatch,
  CurrentHistoryMismatch,
  InvalidDeclaredPrefix,
  ActualPrefixMismatch,
};
/// Authenticate the complete old prefix and exactly the indicated number of
/// already sent new suffix packets. Fresh sensor/world/context and next-send
/// clock/slew checks remain separate obligations.
PrefixReason check_actual_publication_prefix(
  const applied::ScheduledCertificate &certificate,
  const vehicle::PublishedInputLedger &ledger,
  const vehicle::PublishedInputLedger::Snapshot &original_cursor,
  const vehicle::ObservationProvenance &fresh,
  const std::vector<std::optional<vehicle::PublishedProgramSource>> &prior_sources,
  std::size_t already_published_suffix_packets);

enum class CurrentWorldReason {
  Current,
  InvalidContext,
  InvalidFollowObservation,
  FollowOriginUnavailable,
  PhysicalRejected,
};
struct CurrentWorldCheck {
  CurrentWorldReason reason{CurrentWorldReason::InvalidContext};
  applied::Reason physical_reason{applied::Reason::InvalidWorld};
  double rejected_sec{std::numeric_limits<double>::quiet_NaN()};
  std::string rejected_peer_id;
  double minimum_peer_clearance_m{std::numeric_limits<double>::infinity()};
  double minimum_follow_gap_m{std::numeric_limits<double>::infinity()};
  std::size_t checked_samples{};
};
/// Recheck unchanged original numerical body/corner ranges from fresh.now to
/// original rest under the fresh world. Fresh prefix binding is required; no
/// fresh population is created. Session/mission/schema/horizon/bounds/cost-policy
/// adoption context, measurement consistency, ledger and final clocks/slew must
/// also pass before any actual dispatch.
CurrentWorldCheck recheck_remaining_world(
  const applied::ScheduledCertificate &certificate, const retained::Request &fresh);

} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled
