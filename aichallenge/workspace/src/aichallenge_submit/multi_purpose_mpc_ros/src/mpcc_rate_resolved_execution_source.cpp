#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_source.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_execution_source
{

namespace
{

std::optional<double> sample_path_distance_at_elapsed_time(
  const Source & source, const double elapsed_sec) noexcept
{
  if (
    !std::isfinite(elapsed_sec) || elapsed_sec < 0.0 ||
    source.elapsed_time_sec.empty() ||
    source.elapsed_time_sec.size() != source.path_distance_m.size())
  {
    return std::nullopt;
  }
  const auto upper = std::lower_bound(
    source.elapsed_time_sec.begin(), source.elapsed_time_sec.end(), elapsed_sec);
  if (upper == source.elapsed_time_sec.begin()) {
    const double upper_time_sec = source.elapsed_time_sec.front();
    const double upper_path_m = source.path_distance_m.front();
    if (
      !std::isfinite(upper_time_sec) || upper_time_sec <= 0.0 ||
      !std::isfinite(upper_path_m) || upper_path_m < 0.0)
    {
      return std::nullopt;
    }
    return std::clamp(elapsed_sec / upper_time_sec, 0.0, 1.0) * upper_path_m;
  }
  if (upper == source.elapsed_time_sec.end()) {
    return source.path_distance_m.back();
  }
  const auto upper_index = static_cast<std::size_t>(
    std::distance(source.elapsed_time_sec.begin(), upper));
  const auto lower_index = upper_index - 1U;
  const double lower_time_sec = source.elapsed_time_sec[lower_index];
  const double upper_time_sec = source.elapsed_time_sec[upper_index];
  const double lower_path_m = source.path_distance_m[lower_index];
  const double upper_path_m = source.path_distance_m[upper_index];
  if (
    !std::isfinite(lower_time_sec) || !std::isfinite(upper_time_sec) ||
    upper_time_sec <= lower_time_sec || !std::isfinite(lower_path_m) ||
    !std::isfinite(upper_path_m) || upper_path_m <= lower_path_m)
  {
    return std::nullopt;
  }
  const double ratio = std::clamp(
    (elapsed_sec - lower_time_sec) / (upper_time_sec - lower_time_sec),
    0.0, 1.0);
  return lower_path_m + ratio * (upper_path_m - lower_path_m);
}

std::optional<double> project_progress_to_path_distance(
  const Source & source, const double target_progress_m,
  const double cursor_path_distance_m) noexcept
{
  if (
    !std::isfinite(target_progress_m) ||
    !std::isfinite(cursor_path_distance_m) || cursor_path_distance_m < 0.0 ||
    source.progress_m.empty() ||
    source.progress_m.size() != source.path_distance_m.size() ||
    !std::isfinite(source.course_progress_origin_m) ||
    !std::isfinite(source.progress_regression_tolerance_m) ||
    source.progress_regression_tolerance_m < 0.0)
  {
    return std::nullopt;
  }

  constexpr double kProjectionToleranceM = 1e-9;
  double selected_path_m = std::numeric_limits<double>::quiet_NaN();
  double selected_cursor_error_m = std::numeric_limits<double>::infinity();
  bool segment_selected = false;
  double previous_progress_m = source.course_progress_origin_m;
  double previous_path_m = 0.0;
  for (std::size_t index = 0U; index < source.progress_m.size(); ++index) {
    const double progress_m = source.progress_m[index];
    const double path_m = source.path_distance_m[index];
    if (
      !std::isfinite(progress_m) || !std::isfinite(path_m) ||
      (index == 0U ? path_m < 0.0 : path_m <= previous_path_m) ||
      progress_m + source.progress_regression_tolerance_m < previous_progress_m)
    {
      return std::nullopt;
    }
    const double lower_progress_m = std::min(previous_progress_m, progress_m);
    const double upper_progress_m = std::max(previous_progress_m, progress_m);
    if (
      target_progress_m + kProjectionToleranceM >= lower_progress_m &&
      target_progress_m <= upper_progress_m + kProjectionToleranceM)
    {
      const double progress_span_m = progress_m - previous_progress_m;
      double candidate_path_m{};
      if (std::abs(progress_span_m) <= kProjectionToleranceM) {
        candidate_path_m = std::clamp(
          cursor_path_distance_m, previous_path_m, path_m);
      } else {
        const double ratio = std::clamp(
          (target_progress_m - previous_progress_m) / progress_span_m,
          0.0, 1.0);
        candidate_path_m =
          previous_path_m + ratio * (path_m - previous_path_m);
      }
      const double cursor_error_m =
        std::abs(candidate_path_m - cursor_path_distance_m);
      if (!segment_selected || cursor_error_m < selected_cursor_error_m) {
        selected_path_m = candidate_path_m;
        selected_cursor_error_m = cursor_error_m;
        segment_selected = true;
      }
    }
    previous_progress_m = progress_m;
    previous_path_m = path_m;
  }
  if (!segment_selected) {
    return std::nullopt;
  }
  return selected_path_m;
}

}  // namespace

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
  result.source.elapsed_time_sec = exact.elapsed_time_sec;
  result.source.lateral_m = exact.lateral_m;
  result.source.progress_m = exact.progress_m;
  result.source.progress_regression_tolerance_m =
    exact.progress_regression_tolerance_m;
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
    case PublishedRejectReason::CourseProgressProjectionUnavailable:
      return "course-progress-projection-unavailable";
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

  const double advanced_course_progress_m = std::max(0.0, advanced_distance_m);
  const auto cursor_path_distance_m = sample_path_distance_at_elapsed_time(
    projected.source, cursor.elapsed_sec);
  const auto advanced_path_distance_m =
    cursor_path_distance_m.has_value() ? project_progress_to_path_distance(
      projected.source,
      projected.source.course_progress_origin_m + advanced_course_progress_m,
      cursor_path_distance_m.value()) : std::nullopt;
  if (!advanced_path_distance_m.has_value()) {
    result.reason =
      PublishedRejectReason::CourseProgressProjectionUnavailable;
    result.published.cursor = cursor;
    return result;
  }

  result.published.source = projected.source;
  result.published.cursor = cursor;
  result.published.advanced_course_progress_m = advanced_course_progress_m;
  result.published.advanced_path_distance_m = advanced_path_distance_m.value();
  result.reason = PublishedRejectReason::None;
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_execution_source
