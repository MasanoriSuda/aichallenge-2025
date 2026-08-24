#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PHYSICAL_WALL_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PHYSICAL_WALL_HPP_

#include "multi_purpose_mpc_ros/mpc_stage_geometry.hpp"
#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_artifact.hpp"
#include "multi_purpose_mpc_ros/race_mpcc_foundation.hpp"
#include "multi_purpose_mpc_ros/recovery_footprint.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_physical_wall
{

namespace artifact = mpcc_rate_resolved_execution_artifact;
namespace contract = mpcc_execution_contract;
namespace race = race_mpcc_foundation;
namespace recovery = recovery_footprint;

struct Identity
{
  artifact::Identity artifact;
  std::uint64_t pose_snapshot_id{};
  std::uint64_t course_frame_window_id{};
  double captured_sec{};
};

bool identity_valid(const Identity & identity) noexcept;
bool same_identity(const Identity & lhs, const Identity & rhs) noexcept;

/// Immutable input for a complete current-world footprint proof. The shared
/// grid owner is required because this object crosses the control-thread
/// lifetime boundary.
struct Snapshot
{
  Identity identity;
  std::shared_ptr<const recovery::OccupancyGrid> wall_grid;
  recovery::FootprintExtents footprint;
  recovery::Pose2D current_pose;
  race::ExactPhysicalExecutionTrajectory trajectory;
  std::vector<mpc_stage_geometry::CourseFrameKnot> course_frame_knots;
  double hard_wall_clearance_m{};
  double bound_tolerance_m{};
  double swept_step_m{};
};

enum class Outcome
{
  InvalidInput,
  AdapterRejected,
  CurrentPoseRejected,
  LateralBoundRejected,
  CourseFrameRejected,
  StageWallRejected,
  SweptWallRejected,
  Accepted,
  Exception,
  Count,
};

const char * to_string(Outcome outcome) noexcept;

struct Result
{
  Identity identity;
  Outcome outcome{Outcome::InvalidInput};
  contract::PhysicalWallCertificateDiagnostic diagnostic;
  double completed_sec{};
  double compute_ms{};
  std::string detail{"not-evaluated"};
};

bool result_valid(const Result & result) noexcept;
Result evaluate(const Snapshot & snapshot);

enum class PublishReason
{
  Accepted,
  InvalidResult,
  SequenceRollback,
  SequenceNotSubmitted,
  Superseded,
  IdentityMismatch,
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
  std::uint64_t superseded_count{};
  std::uint64_t identity_mismatch_count{};
  PublishReason last_reason{PublishReason::InvalidResult};
  bool result_available{false};
};

class Mailbox
{
public:
  bool register_submission(const Identity & identity);
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
  std::uint64_t superseded_count_{};
  std::uint64_t identity_mismatch_count_{};
  PublishReason last_reason_{PublishReason::InvalidResult};
  std::optional<Identity> latest_submitted_identity_;
  std::optional<Result> latest_result_;
};

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_physical_wall

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PHYSICAL_WALL_HPP_
