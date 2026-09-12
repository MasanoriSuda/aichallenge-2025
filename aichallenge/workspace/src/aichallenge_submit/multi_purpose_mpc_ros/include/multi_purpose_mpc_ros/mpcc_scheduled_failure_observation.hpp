#pragma once
#include "multi_purpose_mpc_ros/mpcc_scheduled_dispatch.hpp"

namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot {
struct ScheduledFailureCapture {
  mpcc_rate_resolved_scheduled::Request original;
  std::shared_ptr<const mpcc_rate_resolved_applied_program::ScheduledCertificate> certificate;
  std::shared_ptr<const mpcc_rate_resolved_retained_revalidation::Request> current;
  mpcc_execution_contract::MpccProblemContext current_context;
  mpcc_execution_contract::MpccProblemContext original_proposed_context;
  mpcc_rate_resolved_scheduled::CurrentCheck current_check;
  bool current_check_observed{false};
  std::optional<bool> source_context_active_at_capture;
  std::optional<mpcc_vehicle_model::PublishedInputLedger::Snapshot> original_cursor;
  std::vector<std::optional<mpcc_vehicle_model::PublishedProgramSource>> prior_sources;
  std::optional<std::vector<mpcc_vehicle_model::PublicationTransaction>> transactions;
  /// Actual preceding send, independent of the inspected candidate/cursor.
  std::optional<mpcc_vehicle_model::PublicationTransaction> last_publication;
  std::size_t suffix_index{};
  std::optional<mpcc_vehicle_model::ObservationProvenance> raw_observation;
  std::shared_ptr<const mpcc_rate_resolved_scheduled::CurrentPhysicalProof> current_physical_proof;
  std::shared_ptr<const mpcc_rate_resolved_scheduled::StartingDomainEvidence> starting_domain;
  std::shared_ptr<const mpcc_rate_resolved_scheduled::CurrentDomainProof> current_domain_proof;
  mpcc_rate_resolved_scheduled::DomainUseReason domain_use{mpcc_rate_resolved_scheduled::DomainUseReason::NotNeeded};
  std::string boundary;
  std::string detail;
};
} // namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot
