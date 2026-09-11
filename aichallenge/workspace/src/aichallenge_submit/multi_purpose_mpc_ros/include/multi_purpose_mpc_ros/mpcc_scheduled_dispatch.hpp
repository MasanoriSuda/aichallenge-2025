#pragma once

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_scheduled.hpp"
#include "multi_purpose_mpc_ros/mpcc_starting_domain.hpp"

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled {

struct DispatchResult;


/// A causal planning reservation, never permission to send. Expected prior IDs
/// come from the already published immutable source, before the new job starts.
/// Intent of this already proved programme phase; grants no send permission.
retained::contract::ControlIntent programme_publication_intent(
  const applied::ScheduledCertificate &certificate, std::size_t suffix_index) noexcept;

enum class ReservationStatus { Waiting, Ready, Invalid };
class SourceReservation {
public:
  static std::shared_ptr<const SourceReservation> build(
    std::shared_ptr<const applied::ScheduledCertificate> active, const ContextSnapshot &generation,
    const vehicle::PublishedInputLedger &ledger, std::size_t next_index,
    std::size_t preceding_packet_count, double observed_now_sec, double prediction_delay_sec);
  ReservationStatus status(const vehicle::PublishedInputLedger &ledger,
    const ContextSnapshot &generation, double now_sec) const;
  const vehicle::PublishedInputLedger::Snapshot &cursor() const noexcept { return *cursor_; }
  const vehicle::PublishedInputProgram &prior_program() const noexcept { return active_->suffix().program; }
  const std::vector<std::optional<vehicle::PublishedProgramSource>> &prior_sources() const noexcept { return prior_sources_; }
  std::size_t prior_index() const noexcept { return prior_index_; }
  double first_publication_sec() const noexcept { return first_publication_sec_; }
  double control_origin_sec() const noexcept { return control_origin_sec_; }
private:
  SourceReservation() = default;
  std::shared_ptr<const applied::ScheduledCertificate> active_;
  ContextSnapshot generation_;
  std::optional<vehicle::PublishedInputLedger::Snapshot> cursor_;
  vehicle::PublishedInputProgram remaining_prior_;
  std::vector<std::optional<vehicle::PublishedProgramSource>> prior_sources_;
  std::size_t prior_index_{};
  double observed_sec_{};
  double first_publication_sec_{};
  double publication_deadline_sec_{};
  double control_origin_sec_{};
};

/// Worker numerical evidence bound to this exact immutable source certificate.
/// It has no current-world or publication authority. A pending-prior source
/// uses the complete original composite programme, including delayed prior
/// inputs, with starting times beginning at the new suffix. The independent
/// suffix-only theorem cannot erase those inputs. Follow keeps its original
/// first-window theorem and does not support pending-prior domains.
class StartingDomainEvidence {
public:
  static std::shared_ptr<const StartingDomainEvidence> build(
    std::shared_ptr<const applied::ScheduledCertificate> certificate);
  const std::shared_ptr<const applied::ScheduledCertificate> &certificate() const noexcept { return certificate_; }
  const vehicle::StartingDomainTube &tube() const noexcept { return tube_; }
  double original_rest_sec() const noexcept { return original_rest_sec_; }
  bool first_window_only() const noexcept { return first_window_only_; }
  bool includes_pending_prior() const noexcept { return includes_pending_prior_; }
private:
  StartingDomainEvidence() = default;
  std::shared_ptr<const applied::ScheduledCertificate> certificate_;
  vehicle::StartingDomainTube tube_;
  double original_rest_sec_{};
  bool first_window_only_{true};
  bool includes_pending_prior_{false};
};

enum class DomainUseReason {
  NotNeeded, Missing, UnsupportedSuffix, SourceMismatch, PrefixUnavailable,
  OutsideDomain, FollowNotMonotone, WorldRejected, Accepted
};

/// Only the dispatcher can join fresh authenticated prefix and whole-world
/// evidence to an independently computed starting domain.
class CurrentDomainProof {
public:
  const retained::Request &observed() const noexcept { return observed_; }
  const vehicle::CurrentInputPrefix &prefix() const noexcept { return prefix_; }
  const std::shared_ptr<const StartingDomainEvidence> &evidence() const noexcept { return evidence_; }
  std::uint64_t physical_input_fingerprint() const noexcept { return physical_input_fingerprint_; }
  std::size_t first_suffix_index() const noexcept { return first_suffix_index_; }
private:
  CurrentDomainProof() = default;
  friend DispatchResult prepare_dispatch(
    std::shared_ptr<const applied::ScheduledCertificate>, const retained::Request &,
    const retained::contract::MpccProblemContext &, const ContextSnapshot &,
    const vehicle::PublishedInputLedger &, const vehicle::PublishedInputLedger::Snapshot &,
    const std::vector<std::optional<vehicle::PublishedProgramSource>> &, std::size_t,
    std::shared_ptr<const StartingDomainEvidence>);
  retained::Request observed_;
  vehicle::CurrentInputPrefix prefix_;
  std::shared_ptr<const StartingDomainEvidence> evidence_;
  std::uint64_t physical_input_fingerprint_{};
  std::size_t first_suffix_index_{};
};

/// Independently proved physical population for the exact unsent original
/// programme. It never replaces the original job/programme identity in the
/// actual ledger. Only prepare_dispatch can mint it after current evidence.
class CurrentPhysicalProof {
public:
  const retained::Request &observed() const noexcept { return observed_; }
  const vehicle::PendingInputTube &tube() const noexcept { return tube_; }
  std::uint64_t original_input_fingerprint() const noexcept { return original_input_fingerprint_; }
  std::size_t first_suffix_index() const noexcept { return first_suffix_index_; }
private:
  CurrentPhysicalProof() = default;
  friend DispatchResult prepare_dispatch(
    std::shared_ptr<const applied::ScheduledCertificate>, const retained::Request &,
    const retained::contract::MpccProblemContext &, const ContextSnapshot &,
    const vehicle::PublishedInputLedger &, const vehicle::PublishedInputLedger::Snapshot &,
    const std::vector<std::optional<vehicle::PublishedProgramSource>> &, std::size_t,
    std::shared_ptr<const StartingDomainEvidence>);
  retained::Request observed_;
  vehicle::PendingInputTube tube_;
  std::uint64_t original_input_fingerprint_{};
  std::size_t first_suffix_index_{};
};

/// Immutable candidate from one current observation and authenticated ledger
/// prefix. It permits no send until matches_before_publication() succeeds at
/// the actual publisher boundary. Only the single control thread may inspect
/// the live ledger; a worker must never own this object as publication authority.
class DispatchCandidate {
public:
  const vehicle::PublishedCommand &packet() const noexcept { return packet_; }
  double physical_steering_rad() const noexcept { return physical_steering_rad_; }
  double predicted_speed_mps() const noexcept { return predicted_speed_mps_; }
  std::uint64_t dispatch_decision_id() const noexcept { return dispatch_decision_id_; }
  const vehicle::PublishedProgramSource &source() const noexcept { return source_; }
  const std::shared_ptr<const applied::ScheduledCertificate> &certificate() const noexcept { return certificate_; }

  const std::shared_ptr<const CurrentPhysicalProof> &current_physical_proof() const noexcept { return current_physical_proof_; }
  const std::shared_ptr<const CurrentDomainProof> &current_domain_proof() const noexcept { return current_domain_proof_; }
  std::uint64_t physical_input_fingerprint() const noexcept {
    if (current_domain_proof_) return current_domain_proof_->physical_input_fingerprint();
    return current_physical_proof_ ? current_physical_proof_->tube().numerical.context_fingerprint : certificate_->tube().context_fingerprint;
  }
  retained::contract::CanonicalNormalCommand canonical_command() const;
  std::shared_ptr<const retained::contract::PublishedScheduledIdentity> publication_identity(const vehicle::PublishedInputLedger &ledger, const ContextSnapshot &current_generation) const;

  /// The creation clock comes from the immutable proof's original observation.
  /// A current callback may enter inside the predeclared window; it may not
  /// choose a new appointment. Both actual endpoints remain strictly bounded.
  /// Check unchanged authenticated history, generation, decision, exact wire,
  /// strict integer window and steering slew from the predecessor's recorded
  /// publication epoch (including its existing causal floor). No causal floor, extra tolerance or retiming.
  bool matches_before_publication(const vehicle::PublishedInputLedger &ledger, const ContextSnapshot &current_generation, std::uint64_t dispatch_decision_id, double before_clock_sec, double wire_acceleration_mps2, double wire_steering_rad) const;

  /// The one actual send must be in the ledger with this intended source and
  /// both valid raw endpoints. Failure is a detected violation, never a reason
  /// to erase the send or to authorize it retrospectively.
  bool matches_after_publication(const vehicle::PublishedInputLedger &ledger, const ContextSnapshot &current_generation) const;

private:
  DispatchCandidate() = default;
  friend DispatchResult prepare_dispatch(
    std::shared_ptr<const applied::ScheduledCertificate>, const retained::Request &,
    const retained::contract::MpccProblemContext &, const ContextSnapshot &,
    const vehicle::PublishedInputLedger &, const vehicle::PublishedInputLedger::Snapshot &,
    const std::vector<std::optional<vehicle::PublishedProgramSource>> &, std::size_t,
    std::shared_ptr<const StartingDomainEvidence>);
  bool clock_and_slew_match(double before_clock_sec, double after_clock_sec) const noexcept;
  std::shared_ptr<const applied::ScheduledCertificate> certificate_;
  std::shared_ptr<const CurrentPhysicalProof> current_physical_proof_;
  std::shared_ptr<const CurrentDomainProof> current_domain_proof_;
  std::optional<vehicle::PublishedInputLedger::Snapshot> ledger_cursor_;
  ContextSnapshot current_generation_;
  vehicle::PublishedCommand packet_;
  vehicle::PublishedProgramSource source_;
  std::size_t packet_index_{};
  std::uint64_t dispatch_decision_id_{};
  double observed_sec_{};
  double physical_steering_rad_{};
  double predicted_speed_mps_{};
  double previous_wire_steering_rad_{};
  double previous_publication_sec_{};
};

enum class DispatchReason {
  Candidate,
  MissingCertificate,
  InvalidPacket,
  CurrentEvidenceRejected,
  InvalidPredecessor,
  RestExpired,
};
struct DispatchResult {
  DispatchReason reason{DispatchReason::MissingCertificate};
  CurrentCheck current;
  DomainUseReason domain_use{DomainUseReason::NotNeeded};
  std::shared_ptr<const DispatchCandidate> candidate;
};

/// The next index equals the number of actual suffix sends, verified against
/// the full original cursor and preceding sources. The fresh point prefix must
/// consider that exact next wire payload, not a newly sampled source command.
DispatchResult prepare_dispatch(
  std::shared_ptr<const applied::ScheduledCertificate> certificate,
  const retained::Request &fresh, const retained::contract::MpccProblemContext &fresh_problem,
  const ContextSnapshot &current_generation, const vehicle::PublishedInputLedger &ledger,
  const vehicle::PublishedInputLedger::Snapshot &original_cursor,
  const std::vector<std::optional<vehicle::PublishedProgramSource>> &prior_sources,
  std::size_t already_published_suffix_packets,
  std::shared_ptr<const StartingDomainEvidence> domain = {});

} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled
