#pragma once
#include "multi_purpose_mpc_ros/mpcc_scheduled_dispatch.hpp"

namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot {
struct ScheduledFailureCapture {
  mpcc_rate_resolved_scheduled::Request original;
  std::shared_ptr<const mpcc_rate_resolved_applied_program::ScheduledCertificate> certificate;
  std::shared_ptr<const mpcc_rate_resolved_retained_revalidation::Request> current;
  mpcc_execution_contract::MpccProblemContext current_context;
  mpcc_rate_resolved_scheduled::CurrentCheck current_check;
  std::optional<mpcc_vehicle_model::PublishedInputLedger::Snapshot> original_cursor;
  std::optional<std::vector<mpcc_vehicle_model::PublicationTransaction>> transactions;
  std::size_t suffix_index{};
  std::string boundary;
  std::string detail;
};
} // namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot
