#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_source.hpp"

#include <cmath>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_execution_source
{

const char * to_string(const RejectReason reason) noexcept
{
  switch (reason) {
    case RejectReason::None: return "none";
    case RejectReason::MissingPlan: return "missing-plan";
    case RejectReason::InvalidCertifiedPlan: return "invalid-certified-plan";
    case RejectReason::UnsupportedIntent: return "unsupported-intent";
    case RejectReason::IntentMismatch: return "intent-mismatch";
    case RejectReason::TargetMismatch: return "target-mismatch";
    case RejectReason::MissionGenerationMismatch:
      return "mission-generation-mismatch";
    case RejectReason::SideMismatch: return "side-mismatch";
    case RejectReason::InvalidSourceTime: return "invalid-source-time";
  }
  return "unknown";
}

Result build(const Request & request)
{
  Result result;
  if (request.plan == nullptr) {
    return result;
  }
  if (
    certified::validate(*request.plan) != certified::RejectReason::None ||
    request.plan->execution_artifact == nullptr ||
    request.plan->physical_snapshot == nullptr)
  {
    result.reason = RejectReason::InvalidCertifiedPlan;
    return result;
  }
  if (
    !contract::canonical_normal_intent_requires_execution_side(request.intent) ||
    request.target_id.empty() || request.mission_generation == 0U ||
    (request.side_sign != -1 && request.side_sign != 1))
  {
    result.reason = RejectReason::UnsupportedIntent;
    return result;
  }

  const auto & artifact = *request.plan->execution_artifact;
  const auto & context = artifact.identity.source_context;
  if (context.intent != request.intent) {
    result.reason = RejectReason::IntentMismatch;
    return result;
  }
  if (context.target_id != request.target_id) {
    result.reason = RejectReason::TargetMismatch;
    return result;
  }
  if (context.intent_generation != request.mission_generation) {
    result.reason = RejectReason::MissionGenerationMismatch;
    return result;
  }
  if (context.execution_side_sign != request.side_sign) {
    result.reason = RejectReason::SideMismatch;
    return result;
  }
  if (
    !std::isfinite(artifact.identity.snapshot_sec) ||
    !std::isfinite(artifact.completed_sec) ||
    artifact.completed_sec + 1e-9 < artifact.identity.snapshot_sec)
  {
    result.reason = RejectReason::InvalidSourceTime;
    return result;
  }

  const auto & exact = request.plan->physical_snapshot->trajectory;
  result.source.artifact_sequence = artifact.identity.sequence;
  result.source.source_context = context;
  result.source.source_snapshot_sec = artifact.identity.snapshot_sec;
  result.source.source_completed_sec = artifact.completed_sec;
  result.source.course_progress_origin_m = exact.progress_origin_m;
  result.source.minimum_lateral_bound_reserve_m =
    exact.minimum_lateral_bound_reserve_m;
  result.source.path_distance_m = exact.path_distance_m;
  result.source.lateral_m = exact.lateral_m;
  result.source.progress_m = exact.progress_m;
  result.reason = RejectReason::None;
  return result;
}

const char * to_string(const PublishedRejectReason reason) noexcept
{
  switch (reason) {
    case PublishedRejectReason::None: return "none";
    case PublishedRejectReason::SourceRejected: return "source-rejected";
    case PublishedRejectReason::InvalidExecutionClock:
      return "invalid-execution-clock";
    case PublishedRejectReason::CursorUnavailable:
      return "cursor-unavailable";
    case PublishedRejectReason::InvalidCourseProgress:
      return "invalid-course-progress";
    case PublishedRejectReason::CourseProgressRegressed:
      return "course-progress-regressed";
  }
  return "unknown";
}

PublishedResult build_published(const PublishedRequest & request)
{
  namespace retained = mpcc_rate_resolved_retained_revalidation;
  PublishedResult result;
  const auto projected = build(request.identity);
  result.source_reason = projected.reason;
  if (!projected.accepted()) {
    return result;
  }
  if (
    !std::isfinite(request.current_control_origin_sec) ||
    request.current_control_origin_sec < 0.0 ||
    !std::isfinite(request.first_published_control_origin_sec) ||
    request.first_published_control_origin_sec < 0.0 ||
    !std::isfinite(request.first_published_artifact_elapsed_sec) ||
    request.first_published_artifact_elapsed_sec < 0.0 ||
    request.current_control_origin_sec + 1e-9 <
    request.first_published_control_origin_sec)
  {
    result.reason = PublishedRejectReason::InvalidExecutionClock;
    return result;
  }
  const auto & artifact = *request.identity.plan->execution_artifact;
  const auto cursor = retained::resolve_execution_cursor(
    artifact, request.current_control_origin_sec,
    retained::ExecutionClock{
      retained::ExecutionClockKind::PublishedPlan,
      request.first_published_control_origin_sec,
      request.first_published_artifact_elapsed_sec});
  if (!cursor.available) {
    result.reason = PublishedRejectReason::CursorUnavailable;
    result.published.cursor = cursor;
    return result;
  }
  if (
    !std::isfinite(request.measured_course_progress_m) ||
    !std::isfinite(projected.source.course_progress_origin_m) ||
    (request.circular &&
    (!std::isfinite(request.path_length_m) || request.path_length_m <= 0.0)))
  {
    result.reason = PublishedRejectReason::InvalidCourseProgress;
    return result;
  }

  double advanced_distance_m =
    request.measured_course_progress_m -
    projected.source.course_progress_origin_m;
  if (request.circular) {
    const double half_path_length_m = 0.5 * request.path_length_m;
    while (advanced_distance_m > half_path_length_m) {
      advanced_distance_m -= request.path_length_m;
    }
    while (advanced_distance_m < -half_path_length_m) {
      advanced_distance_m += request.path_length_m;
    }
  }
  constexpr double kProgressToleranceM = 1e-3;
  if (!std::isfinite(advanced_distance_m)) {
    result.reason = PublishedRejectReason::InvalidCourseProgress;
    return result;
  }
  if (advanced_distance_m < -kProgressToleranceM) {
    result.reason = PublishedRejectReason::CourseProgressRegressed;
    return result;
  }

  result.published.source = projected.source;
  result.published.cursor = cursor;
  result.published.advanced_distance_m = std::max(0.0, advanced_distance_m);
  result.reason = PublishedRejectReason::None;
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_execution_source
