#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_physical_wall
{

namespace
{

using SteadyClock = std::chrono::steady_clock;

bool finite_pose(const recovery::Pose2D & pose) noexcept
{
  return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
         std::isfinite(pose.yaw_rad);
}

Result finish(
  Result result, const SteadyClock::time_point started,
  const double captured_sec)
{
  result.compute_ms = std::chrono::duration<double, std::milli>(
    SteadyClock::now() - started).count();
  result.completed_sec = captured_sec + result.compute_ms * 1.0e-3;
  return result;
}

}  // namespace

bool identity_valid(const Identity & identity) noexcept
{
  return artifact::identity_valid(identity.artifact) &&
         identity.pose_snapshot_id != 0U &&
         identity.course_frame_window_id != 0U &&
         std::isfinite(identity.captured_sec) &&
         identity.captured_sec >= identity.artifact.snapshot_sec;
}

bool same_identity(const Identity & lhs, const Identity & rhs) noexcept
{
  return
    lhs.artifact.sequence == rhs.artifact.sequence &&
    lhs.artifact.decision_id == rhs.artifact.decision_id &&
    lhs.artifact.source_problem_fingerprint ==
    rhs.artifact.source_problem_fingerprint &&
    lhs.artifact.stage_geometry_id == rhs.artifact.stage_geometry_id &&
    lhs.artifact.intent == rhs.artifact.intent &&
    lhs.artifact.snapshot_sec == rhs.artifact.snapshot_sec &&
    lhs.pose_snapshot_id == rhs.pose_snapshot_id &&
    lhs.course_frame_window_id == rhs.course_frame_window_id &&
    lhs.captured_sec == rhs.captured_sec;
}

bool snapshot_valid(const Snapshot & snapshot) noexcept
{
  const auto trajectory_validation =
    race::validate_exact_physical_execution_trajectory(snapshot.trajectory);
  return identity_valid(snapshot.identity) && snapshot.wall_grid != nullptr &&
         snapshot.wall_grid->valid() && snapshot.footprint.valid() &&
         finite_pose(snapshot.current_pose) && trajectory_validation.complete &&
         snapshot.course_frame_knots.size() >= 2U &&
         std::isfinite(snapshot.hard_wall_clearance_m) &&
         snapshot.hard_wall_clearance_m >= 0.0 &&
         std::isfinite(snapshot.bound_tolerance_m) &&
         snapshot.bound_tolerance_m >= 0.0 &&
         std::isfinite(snapshot.swept_step_m) && snapshot.swept_step_m > 0.0;
}

const char * to_string(const Outcome outcome) noexcept
{
  switch (outcome) {
    case Outcome::InvalidInput: return "invalid-input";
    case Outcome::AdapterRejected: return "adapter-rejected";
    case Outcome::CurrentPoseRejected: return "current-pose-rejected";
    case Outcome::LateralBoundRejected: return "lateral-bound-rejected";
    case Outcome::CourseFrameRejected: return "course-frame-rejected";
    case Outcome::StageWallRejected: return "stage-wall-rejected";
    case Outcome::SweptWallRejected: return "swept-wall-rejected";
    case Outcome::Accepted: return "accepted";
    case Outcome::Exception: return "exception";
    case Outcome::Count: break;
  }
  return "unknown";
}

bool result_valid(const Result & result) noexcept
{
  const auto outcome_index = static_cast<std::size_t>(result.outcome);
  return identity_valid(result.identity) && outcome_index <
         static_cast<std::size_t>(Outcome::Count) &&
         std::isfinite(result.completed_sec) &&
         result.completed_sec >= result.identity.captured_sec &&
         std::isfinite(result.compute_ms) && result.compute_ms >= 0.0;
}

Result evaluate(const Snapshot & snapshot)
{
  const auto started = SteadyClock::now();
  Result result;
  result.identity = snapshot.identity;
  auto reject = [&](const Outcome outcome,
      const contract::PhysicalWallCertificateReason reason,
      std::string detail) {
      result.outcome = outcome;
      result.diagnostic.reason = reason;
      result.detail = std::move(detail);
      return finish(std::move(result), started, snapshot.identity.captured_sec);
    };

  if (!snapshot_valid(snapshot))
  {
    return reject(
      Outcome::InvalidInput,
      contract::PhysicalWallCertificateReason::InvalidInput,
      "rate-resolved physical wall snapshot invalid");
  }

  auto clearance_footprint = snapshot.footprint;
  clearance_footprint.left_extent_m += snapshot.hard_wall_clearance_m;
  clearance_footprint.right_extent_m += snapshot.hard_wall_clearance_m;

  result.diagnostic.stage_index = -1;
  result.diagnostic.waypoint_id = -1;
  result.diagnostic.pose_x_m = snapshot.current_pose.x_m;
  result.diagnostic.pose_y_m = snapshot.current_pose.y_m;
  result.diagnostic.pose_yaw_rad = snapshot.current_pose.yaw_rad;
  const auto current_sample = recovery::sample_footprint(
    *snapshot.wall_grid, clearance_footprint, snapshot.current_pose);
  result.diagnostic.out_of_map = current_sample.out_of_map;
  result.diagnostic.contact_cell_count = current_sample.contact_cells.size();
  if (
    !current_sample.valid || current_sample.out_of_map ||
    !current_sample.contact_cells.empty())
  {
    const auto reason = !current_sample.valid || current_sample.out_of_map ?
      contract::PhysicalWallCertificateReason::CurrentPoseWallSampleUnavailable :
      contract::PhysicalWallCertificateReason::CurrentPoseHardWallContact;
    return reject(
      Outcome::CurrentPoseRejected, reason,
      !current_sample.valid || current_sample.out_of_map ?
      "current pose wall sample unavailable" :
      "current pose hard wall contact");
  }

  const std::size_t stage_count = snapshot.trajectory.lateral_m.size();
  std::vector<recovery::Pose2D> swept_path;
  swept_path.reserve(stage_count + 1U);
  swept_path.push_back(snapshot.current_pose);
  std::vector<int> waypoint_ids;
  waypoint_ids.reserve(stage_count);
  for (std::size_t stage = 0U; stage < stage_count; ++stage) {
    const double lateral_m = snapshot.trajectory.lateral_m[stage];
    const double lag_m = snapshot.trajectory.lag_m[stage];
    const double lower_m = snapshot.trajectory.lateral_lower_m[stage];
    const double upper_m = snapshot.trajectory.lateral_upper_m[stage];
    result.diagnostic.stage_index = static_cast<int>(stage);
    result.diagnostic.path_distance_m =
      snapshot.trajectory.path_distance_m[stage];
    result.diagnostic.lateral_m = lateral_m;
    result.diagnostic.lag_m = lag_m;
    result.diagnostic.lower_bound_m = lower_m;
    result.diagnostic.upper_bound_m = upper_m;
    result.diagnostic.bound_reserve_m = std::min(
      lateral_m - lower_m, upper_m - lateral_m);
    result.diagnostic.reference_progress_m =
      snapshot.trajectory.progress_origin_m +
      snapshot.trajectory.path_distance_m[stage];
    result.diagnostic.solved_progress_m =
      snapshot.trajectory.progress_m[stage];
    result.diagnostic.progress_delta_m =
      result.diagnostic.solved_progress_m -
      result.diagnostic.reference_progress_m;
    result.diagnostic.heading_offset_rad =
      snapshot.trajectory.heading_offset_rad[stage];
    if (
      lateral_m < lower_m - snapshot.bound_tolerance_m ||
      lateral_m > upper_m + snapshot.bound_tolerance_m)
    {
      return reject(
        Outcome::LateralBoundRejected,
        contract::PhysicalWallCertificateReason::LateralBoundViolation,
        "current wall bounds rejected rate-resolved solution");
    }

    const auto course_frame = mpc_stage_geometry::sample_course_frame(
      snapshot.course_frame_knots, snapshot.trajectory.progress_m[stage],
      snapshot.bound_tolerance_m);
    if (!course_frame.has_value()) {
      return reject(
        Outcome::CourseFrameRejected,
        contract::PhysicalWallCertificateReason::CourseFrameUnavailable,
        "rate-resolved progress course frame unavailable");
    }
    const int waypoint_id = course_frame->interpolation_ratio < 0.5 ?
      course_frame->lower_waypoint : course_frame->upper_waypoint;
    result.diagnostic.waypoint_id = waypoint_id;
    const auto pose = contract::reconstruct_planar_pose_from_frenet(
      contract::PlanarPose{
        course_frame->x_m, course_frame->y_m, course_frame->heading_rad},
      contract::FrenetPose{
        lateral_m, lag_m,
        snapshot.trajectory.heading_offset_rad[stage]});
    if (!pose.has_value()) {
      return reject(
        Outcome::InvalidInput,
        contract::PhysicalWallCertificateReason::InvalidInput,
        "rate-resolved Frenet pose reconstruction invalid");
    }
    const recovery::Pose2D world_pose{
      pose->x_m, pose->y_m, pose->yaw_rad};
    result.diagnostic.pose_x_m = world_pose.x_m;
    result.diagnostic.pose_y_m = world_pose.y_m;
    result.diagnostic.pose_yaw_rad = world_pose.yaw_rad;
    const auto endpoint_sample = recovery::sample_footprint(
      *snapshot.wall_grid, clearance_footprint, world_pose);
    result.diagnostic.out_of_map = endpoint_sample.out_of_map;
    result.diagnostic.contact_cell_count =
      endpoint_sample.contact_cells.size();
    if (
      !endpoint_sample.valid || endpoint_sample.out_of_map ||
      !endpoint_sample.contact_cells.empty())
    {
      const auto reason = !endpoint_sample.valid || endpoint_sample.out_of_map ?
        contract::PhysicalWallCertificateReason::WallSampleUnavailable :
        contract::PhysicalWallCertificateReason::HardWallContact;
      return reject(
        Outcome::StageWallRejected, reason,
        !endpoint_sample.valid || endpoint_sample.out_of_map ?
        "rate-resolved stage wall sample unavailable" :
        "rate-resolved stage hard wall contact");
    }
    waypoint_ids.push_back(waypoint_id);
    swept_path.push_back(world_pose);
  }

  const auto swept = recovery::evaluate_clear_footprint_path(
    *snapshot.wall_grid, clearance_footprint, swept_path,
    snapshot.swept_step_m);
  if (!swept.valid || !swept.clear) {
    result.diagnostic.swept_rejected_path_index = swept.rejected_path_index;
    result.diagnostic.swept_checked_pose_count = swept.checked_pose_count;
    result.diagnostic.swept_rejected_substep =
      swept.rejected_segment_substep;
    result.diagnostic.swept_rejected_subdivision_count =
      swept.rejected_segment_subdivision_count;
    result.diagnostic.swept_rejected_segment_ratio =
      swept.rejected_segment_ratio;
    const auto origin = contract::resolve_swept_path_failure_origin(
      swept.rejected_path_index, stage_count);
    if (origin.origin == contract::PhysicalWallPathFailureOrigin::CurrentPose) {
      result.diagnostic.reason =
        contract::PhysicalWallCertificateReason::CurrentPoseHardWallContact;
      result.diagnostic.stage_index = -1;
      result.diagnostic.waypoint_id = -1;
    } else if (
      origin.origin == contract::PhysicalWallPathFailureOrigin::HorizonStage)
    {
      const auto stage = static_cast<std::size_t>(origin.stage_index);
      result.diagnostic.reason =
        contract::PhysicalWallCertificateReason::SweptPathViolation;
      result.diagnostic.stage_index = origin.stage_index;
      result.diagnostic.waypoint_id = waypoint_ids[stage];
      result.diagnostic.path_distance_m =
        snapshot.trajectory.path_distance_m[stage];
      result.diagnostic.lateral_m = snapshot.trajectory.lateral_m[stage];
      result.diagnostic.lag_m = snapshot.trajectory.lag_m[stage];
      result.diagnostic.heading_offset_rad =
        snapshot.trajectory.heading_offset_rad[stage];
      result.diagnostic.lower_bound_m =
        snapshot.trajectory.lateral_lower_m[stage];
      result.diagnostic.upper_bound_m =
        snapshot.trajectory.lateral_upper_m[stage];
      result.diagnostic.bound_reserve_m = std::min(
        result.diagnostic.lateral_m - result.diagnostic.lower_bound_m,
        result.diagnostic.upper_bound_m - result.diagnostic.lateral_m);
      result.diagnostic.reference_progress_m =
        snapshot.trajectory.progress_origin_m +
        snapshot.trajectory.path_distance_m[stage];
      result.diagnostic.solved_progress_m =
        snapshot.trajectory.progress_m[stage];
      result.diagnostic.progress_delta_m =
        result.diagnostic.solved_progress_m -
        result.diagnostic.reference_progress_m;
    } else {
      result.diagnostic.reason =
        contract::PhysicalWallCertificateReason::InvalidInput;
      result.diagnostic.stage_index = -1;
      result.diagnostic.waypoint_id = -1;
    }
    if (swept.rejected_pose_available) {
      result.diagnostic.pose_x_m = swept.rejected_pose.x_m;
      result.diagnostic.pose_y_m = swept.rejected_pose.y_m;
      result.diagnostic.pose_yaw_rad = swept.rejected_pose.yaw_rad;
    }
    std::ostringstream detail;
    detail << "rate-resolved swept wall path " <<
      recovery::to_string(swept.reason) << " at path index " <<
      swept.rejected_path_index << ", checked_poses=" <<
      swept.checked_pose_count << ", segment_ratio=" <<
      swept.rejected_segment_ratio;
    result.outcome = Outcome::SweptWallRejected;
    result.detail = detail.str();
    return finish(std::move(result), started, snapshot.identity.captured_sec);
  }

  result.outcome = Outcome::Accepted;
  result.diagnostic.reason =
    contract::PhysicalWallCertificateReason::Accepted;
  result.detail = "physical solution horizon accepted, " +
    contract::format_physical_wall_certificate_diagnostic(result.diagnostic);
  return finish(std::move(result), started, snapshot.identity.captured_sec);
}

const char * to_string(const PublishReason reason) noexcept
{
  switch (reason) {
    case PublishReason::Accepted: return "accepted";
    case PublishReason::InvalidResult: return "invalid-result";
    case PublishReason::SequenceRollback: return "sequence-rollback";
    case PublishReason::SequenceNotSubmitted: return "sequence-not-submitted";
    case PublishReason::Superseded: return "superseded";
    case PublishReason::IdentityMismatch: return "identity-mismatch";
  }
  return "unknown";
}

bool Mailbox::register_submission(const Identity & identity)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    !identity_valid(identity) ||
    identity.artifact.sequence <= latest_submitted_sequence_)
  {
    return false;
  }
  latest_submitted_sequence_ = identity.artifact.sequence;
  latest_submitted_identity_ = identity;
  return true;
}

PublishReason Mailbox::publish(Result result)
{
  if (!result_valid(result)) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_result_count_;
    last_reason_ = PublishReason::InvalidResult;
    return last_reason_;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto sequence = result.identity.artifact.sequence;
  if (sequence <= latest_published_sequence_) {
    ++sequence_rollback_count_;
    last_reason_ = PublishReason::SequenceRollback;
    return last_reason_;
  }
  if (sequence > latest_submitted_sequence_) {
    ++sequence_not_submitted_count_;
    last_reason_ = PublishReason::SequenceNotSubmitted;
    return last_reason_;
  }
  if (sequence < latest_submitted_sequence_) {
    ++superseded_count_;
    last_reason_ = PublishReason::Superseded;
    return last_reason_;
  }
  if (
    !latest_submitted_identity_.has_value() ||
    !same_identity(result.identity, latest_submitted_identity_.value()))
  {
    ++identity_mismatch_count_;
    last_reason_ = PublishReason::IdentityMismatch;
    return last_reason_;
  }
  latest_published_sequence_ = sequence;
  latest_result_ = std::move(result);
  ++accepted_count_;
  last_reason_ = PublishReason::Accepted;
  return last_reason_;
}

std::optional<Result> Mailbox::latest_after(
  const std::uint64_t consumed_sequence) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    !latest_result_.has_value() ||
    latest_result_->identity.artifact.sequence <= consumed_sequence)
  {
    return std::nullopt;
  }
  return latest_result_;
}

MailboxState Mailbox::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return MailboxState{
    latest_submitted_sequence_, latest_published_sequence_, accepted_count_,
    invalid_result_count_, sequence_rollback_count_,
    sequence_not_submitted_count_, superseded_count_,
    identity_mismatch_count_, last_reason_, latest_result_.has_value()};
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_physical_wall
