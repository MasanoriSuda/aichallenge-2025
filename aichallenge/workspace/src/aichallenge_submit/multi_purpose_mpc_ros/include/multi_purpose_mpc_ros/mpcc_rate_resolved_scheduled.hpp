#pragma once

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"

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

} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled
