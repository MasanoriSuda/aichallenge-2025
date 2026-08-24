#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_SHADOW_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_SHADOW_HPP_

#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_adapter.hpp"
#include "multi_purpose_mpc_ros/persistent_osqp.hpp"

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_shadow
{

struct Identity
{
  std::uint64_t sequence{};
  std::uint64_t decision_id{};
  std::uint64_t source_problem_fingerprint{};
  std::uint64_t stage_geometry_id{};
  mpcc_execution_contract::ControlIntent intent{
    mpcc_execution_contract::ControlIntent::Unknown};
  double snapshot_sec{};
};

struct Snapshot
{
  Identity identity;
  mpcc_rate_resolved_adapter::Request request;
  double publication_interval_sec{};
};

enum class Outcome
{
  BuildRejected,
  AssemblyRejected,
  SolveRejected,
  NonfiniteResult,
  ActuationSampleRejected,
  Solved,
  Exception,
  Count,
};

const char * to_string(Outcome outcome) noexcept;

struct Result
{
  Identity identity;
  Outcome outcome{Outcome::BuildRejected};
  double completed_sec{};
  double compute_ms{};
  bool adapter_built{false};
  bool assembled{false};
  bool solve_attempted{false};
  bool solved{false};
  bool finite{false};
  bool constraints_satisfied{false};
  bool actuation_sampled{false};
  mpcc_rate_resolved::ActuationSampleReason actuation_sample_reason{
    mpcc_rate_resolved::ActuationSampleReason::Count};
  double first_acceleration_mps2{};
  double first_steering_rate_radps{};
  double first_virtual_progress_speed_mps{};
  double initial_steering_rad{};
  double solver_initial_steering_rad{};
  double sampled_steering_rad{};
  double calculated_terminal_steering_rad{};
  double first_stage_duration_sec{};
  double publication_interval_sec{};
  double maximum_abs_steering_rad{};
  double maximum_abs_steering_rate_radps{};
  double first_steering_rate_physical_lower_radps{};
  double first_steering_rate_physical_upper_radps{};
  double first_steering_rate_solver_lower_radps{};
  double first_steering_rate_solver_upper_radps{};
  double first_steering_rate_certificate_margin_radps{};
  std::size_t certified_stage_count{};
  std::size_t sampled_stage_index{};
  double sampled_stage_elapsed_sec{};
  double certified_horizon_duration_sec{};
  double sampled_curvature_radpm{};
  double terminal_velocity_mps{};
  double terminal_progress_m{};
  double terminal_steering_rad{};
  double maximum_constraint_violation{};
  double maximum_normalized_constraint_violation{};
  int maximum_normalized_constraint_row{-1};
  persistent_osqp::SolveTelemetry solver;
  std::string detail;
};

bool identity_valid(const Identity & identity) noexcept;
bool result_valid(const Result & result) noexcept;

/// Dedicated numerical owner for the rate-resolved shadow. Calls are
/// serialized and deliberately cold-started until an exact six-state
/// progress-rebase warm-start contract is approved.
class SolverContext
{
public:
  Result evaluate(const Snapshot & snapshot);

private:
  std::mutex mutex_;
  persistent_osqp::PersistentOsqpSolver solver_{
    persistent_osqp::ConstraintPreconditioningPolicy::RowToleranceNormalized};
};

enum class PublishReason
{
  Accepted,
  InvalidResult,
  SequenceRollback,
  SequenceNotSubmitted,
};

const char * to_string(PublishReason reason) noexcept;

struct MailboxState
{
  std::uint64_t latest_submitted_sequence{};
  std::uint64_t latest_published_sequence{};
  std::uint64_t accepted_count{};
  std::uint64_t invalid_result_count{};
  std::uint64_t sequence_rollback_count{};
  std::uint64_t sequence_not_submitted_count{};
  PublishReason last_reason{PublishReason::InvalidResult};
  bool result_available{false};
};

/// Observation-only monotonic transport. It intentionally has no plan-store
/// or authority API, so a shadow result cannot become executable by accident.
class Mailbox
{
public:
  bool register_submission(std::uint64_t sequence);
  PublishReason publish(Result result);
  std::optional<Result> latest_after(std::uint64_t consumed_sequence) const;
  MailboxState state() const;

private:
  mutable std::mutex mutex_;
  std::uint64_t latest_submitted_sequence_{};
  std::uint64_t latest_published_sequence_{};
  std::uint64_t accepted_count_{};
  std::uint64_t invalid_result_count_{};
  std::uint64_t sequence_rollback_count_{};
  std::uint64_t sequence_not_submitted_count_{};
  PublishReason last_reason_{PublishReason::InvalidResult};
  std::optional<Result> latest_result_;
};

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_shadow

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_SHADOW_HPP_
