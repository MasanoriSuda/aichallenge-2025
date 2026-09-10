#pragma once

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_dynamic_proof.hpp"
#include "multi_purpose_mpc_ros/mpcc_stop_input_program.hpp"

#include <memory>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_retained_revalidation {
struct Request;
struct Proof;
} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_retained_revalidation

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_applied_program {
namespace retained = mpcc_rate_resolved_retained_revalidation;
namespace vehicle = mpcc_vehicle_model;
namespace program = mpcc_stop_input_program;

class Certificate;
struct Result;

/// An immutable applied-input proof bound to the nominal solved source and the
/// complete current-world request. It supplements the nominal nine-state proof;
/// it cannot replace that solve, its intent, or the final command transaction.
class Certificate {
public:
  const program::Prepared &prepared() const noexcept { return prepared_; }
  const vehicle::AppliedInputTube &tube() const noexcept { return tube_; }
  double minimum_peer_clearance_m() const noexcept {
    return minimum_peer_clearance_m_;
  }
  double minimum_follow_gap_m() const noexcept { return minimum_follow_gap_m_; }
  std::size_t checked_samples() const noexcept { return checked_samples_; }
  bool matches(const retained::Request &request) const noexcept;
  bool matches(const retained::Proof &proof) const noexcept;

private:
  Certificate() = default;
  friend Result certify_terminal_stop(const retained::Request &,
                                      const retained::Proof &,
                                      const vehicle::InputApplicationProfile &);
  std::uint64_t nominal_fingerprint_{};
  program::Prepared prepared_;
  vehicle::AppliedInputTube tube_;
  std::shared_ptr<const retained::Request> request_;
  double minimum_peer_clearance_m_{std::numeric_limits<double>::infinity()};
  double minimum_follow_gap_m_{std::numeric_limits<double>::infinity()};
  std::size_t checked_samples_{};
};

enum class Reason {
  Accepted,
  InvalidNominalProof,
  ProgramUnavailable,
  InputPredictionRejected,
  InvalidWorld,
  StateBoundRejected,
  WallRejected,
  PeerRejected,
  FollowProjectionUnavailable,
  FollowGapRejected,
};

struct Result {
  Reason reason{Reason::InvalidNominalProof};
  program::Reason program_reason{program::Reason::InvalidIdentity};
  vehicle::AppliedInputRejectReason prediction_reason{
      vehicle::AppliedInputRejectReason::None};
  std::shared_ptr<const Certificate> certificate;
  double rejected_sec{std::numeric_limits<double>::quiet_NaN()};
  std::string rejected_peer_id;
};

/// Certify exactly the nominal proof's common Stop program, including its
/// already selected first packet. Profile values are explicit empirical input
/// assumptions. No receipt, contact or body-model error guarantee is inferred.
Result certify_terminal_stop(const retained::Request &request,
                             const retained::Proof &nominal,
                             const vehicle::InputApplicationProfile &profile);

} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_applied_program
