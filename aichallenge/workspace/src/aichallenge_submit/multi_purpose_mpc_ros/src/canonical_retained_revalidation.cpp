#include "multi_purpose_mpc_ros/canonical_retained_revalidation.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <type_traits>
#include <utility>

namespace multi_purpose_mpc_ros::canonical_retained_revalidation
{
namespace
{

constexpr double kIdentityTolerance = 1e-9;

bool same_double(const double left, const double right) noexcept
{
  return std::isfinite(left) && std::isfinite(right) &&
         std::abs(left - right) <= kIdentityTolerance;
}

bool nonnegative_finite(const double value) noexcept
{
  return std::isfinite(value) && value >= 0.0;
}

bool same_cursor(
  const plan::CanonicalExecutionCursor & left,
  const plan::CanonicalExecutionCursor & right) noexcept
{
  return left.available == right.available && left.reason == right.reason &&
         left.plan_id == right.plan_id &&
         left.first_control_stage_index == right.first_control_stage_index &&
         left.remaining_control_stage_count ==
         right.remaining_control_stage_count &&
         same_double(left.stage_elapsed_sec, right.stage_elapsed_sec);
}

bool same_state(
  const plan::CanonicalPredictedState & left,
  const plan::CanonicalPredictedState & right) noexcept
{
  return same_double(left.lateral_m, right.lateral_m) &&
         same_double(left.lag_m, right.lag_m) &&
         same_double(left.heading_offset_rad, right.heading_offset_rad) &&
         same_double(left.velocity_mps, right.velocity_mps) &&
         same_double(left.progress_m, right.progress_m);
}

bool same_sample(
  const RetainedStageSample & left,
  const RetainedStageSample & right) noexcept
{
  return left.control_stage_index == right.control_stage_index &&
         left.endpoint_state_index == right.endpoint_state_index &&
         same_double(left.relative_time_sec, right.relative_time_sec) &&
         same_double(left.segment_duration_sec, right.segment_duration_sec) &&
         same_double(
    left.segment_start_progress_m,
    right.segment_start_progress_m) &&
         same_double(left.absolute_progress_m, right.absolute_progress_m) &&
         same_state(left.endpoint, right.endpoint);
}

bool same_window(
  const RetainedExecutionWindow & left,
  const RetainedExecutionWindow & right) noexcept
{
  if (left.plan_id != right.plan_id ||
    !same_cursor(left.cursor, right.cursor) ||
    !same_state(left.expected_current_state, right.expected_current_state) ||
    !same_double(
      left.expected_current_progress_m,
      right.expected_current_progress_m) ||
    left.samples.size() != right.samples.size())
  {
    return false;
  }
  for (std::size_t index = 0U; index < left.samples.size(); ++index) {
    if (!same_sample(left.samples[index], right.samples[index])) {
      return false;
    }
  }
  return true;
}

bool same_current(
  const CurrentExecutionProvenance & left,
  const CurrentExecutionProvenance & right) noexcept
{
  return left.decision_id == right.decision_id && left.intent == right.intent &&
         left.intent_generation == right.intent_generation &&
         left.observation_generation == right.observation_generation &&
         left.stage_geometry_id == right.stage_geometry_id &&
         left.target_obstacle_generation == right.target_obstacle_generation &&
         left.target_id == right.target_id &&
         left.control_pose_id == right.control_pose_id &&
         left.course_frame_window_id == right.course_frame_window_id &&
         left.obstacle_tube_id == right.obstacle_tube_id &&
         same_double(left.observation_sec, right.observation_sec) &&
         same_double(left.path_length_m, right.path_length_m) &&
         left.circular == right.circular;
}

class FingerprintBuilder
{
public:
  void add_bool(const bool value) noexcept {add_uint64(value ? 1U : 0U);}

  template<typename EnumT> void add_enum(const EnumT value) noexcept
  {
    static_assert(std::is_enum<EnumT>::value, "EnumT must be an enum");
    add_uint64(static_cast<std::uint64_t>(value));
  }

  void add_size(const std::size_t value) noexcept
  {
    add_uint64(static_cast<std::uint64_t>(value));
  }

  void add_long(const long value) noexcept
  {
    std::uint64_t bits{};
    static_assert(sizeof(bits) >= sizeof(value), "unsupported long width");
    std::memcpy(&bits, &value, sizeof(value));
    add_uint64(bits);
  }

  void add_uint64(const std::uint64_t value) noexcept
  {
    for (std::size_t shift = 0U; shift < sizeof(value); ++shift) {
      const auto byte = static_cast<unsigned char>(value >> (8U * shift));
      hash_ ^= static_cast<std::uint64_t>(byte);
      hash_ *= 1099511628211ULL;
    }
  }

  void add_double(const double value) noexcept
  {
    std::uint64_t bits{};
    static_assert(sizeof(bits) == sizeof(value), "unexpected double width");
    std::memcpy(&bits, &value, sizeof(value));
    add_uint64(bits);
  }

  void add_string(const std::string & value) noexcept
  {
    add_size(value.size());
    for (const unsigned char byte : value) {
      hash_ ^= static_cast<std::uint64_t>(byte);
      hash_ *= 1099511628211ULL;
    }
  }

  std::uint64_t value() const noexcept {return hash_ == 0U ? 1U : hash_;}

private:
  std::uint64_t hash_{1469598103934665603ULL};
};

void add_current(
  FingerprintBuilder & builder,
  const CurrentExecutionProvenance & current) noexcept
{
  builder.add_uint64(current.decision_id);
  builder.add_enum(current.intent);
  builder.add_uint64(current.intent_generation);
  builder.add_uint64(current.observation_generation);
  builder.add_uint64(current.stage_geometry_id);
  builder.add_uint64(current.target_obstacle_generation);
  builder.add_string(current.target_id);
  builder.add_uint64(current.control_pose_id);
  builder.add_uint64(current.course_frame_window_id);
  builder.add_uint64(current.obstacle_tube_id);
  builder.add_double(current.observation_sec);
  builder.add_double(current.path_length_m);
  builder.add_bool(current.circular);
}

void add_cursor(
  FingerprintBuilder & builder,
  const plan::CanonicalExecutionCursor & cursor) noexcept
{
  builder.add_bool(cursor.available);
  builder.add_enum(cursor.reason);
  builder.add_uint64(cursor.plan_id);
  builder.add_size(cursor.first_control_stage_index);
  builder.add_size(cursor.remaining_control_stage_count);
  builder.add_double(cursor.stage_elapsed_sec);
}

void add_state(
  FingerprintBuilder & builder,
  const plan::CanonicalPredictedState & state) noexcept
{
  builder.add_double(state.lateral_m);
  builder.add_double(state.lag_m);
  builder.add_double(state.heading_offset_rad);
  builder.add_double(state.velocity_mps);
  builder.add_double(state.progress_m);
}

void add_window(
  FingerprintBuilder & builder,
  const RetainedExecutionWindow & window) noexcept
{
  builder.add_uint64(window.plan_id);
  add_cursor(builder, window.cursor);
  add_state(builder, window.expected_current_state);
  builder.add_double(window.expected_current_progress_m);
  builder.add_size(window.samples.size());
  for (const auto & sample : window.samples) {
    builder.add_size(sample.control_stage_index);
    builder.add_size(sample.endpoint_state_index);
    builder.add_double(sample.relative_time_sec);
    builder.add_double(sample.segment_duration_sec);
    builder.add_double(sample.segment_start_progress_m);
    builder.add_double(sample.absolute_progress_m);
    add_state(builder, sample.endpoint);
  }
}

void add_segment(
  FingerprintBuilder & builder,
  const RetainedPathSegmentEvaluation & segment) noexcept
{
  builder.add_uint64(segment.observation_generation);
  builder.add_uint64(segment.stage_geometry_id);
  builder.add_uint64(segment.target_obstacle_generation);
  builder.add_uint64(segment.control_pose_id);
  builder.add_uint64(segment.course_frame_window_id);
  builder.add_uint64(segment.obstacle_tube_id);
  builder.add_double(segment.start_progress_m);
  builder.add_double(segment.end_progress_m);
  builder.add_bool(segment.checked);
  builder.add_bool(segment.wall_clear);
  builder.add_bool(segment.obstacles_clear);
  builder.add_double(segment.minimum_wall_clearance_m);
  builder.add_double(segment.minimum_obstacle_clearance_m);
}

void add_stage_evaluation(
  FingerprintBuilder & builder,
  const RetainedStageSafetyEvaluation & evaluation) noexcept
{
  builder.add_size(evaluation.control_stage_index);
  builder.add_double(evaluation.relative_time_sec);
  builder.add_double(evaluation.segment_duration_sec);
  builder.add_double(evaluation.segment_start_progress_m);
  builder.add_double(evaluation.absolute_progress_m);
  builder.add_uint64(evaluation.observation_generation);
  builder.add_uint64(evaluation.stage_geometry_id);
  builder.add_uint64(evaluation.target_obstacle_generation);
  builder.add_uint64(evaluation.course_frame_window_id);
  builder.add_uint64(evaluation.obstacle_tube_id);
  builder.add_bool(evaluation.course_frame_available);
  builder.add_bool(evaluation.wall_checked);
  builder.add_bool(evaluation.wall_clear);
  builder.add_bool(evaluation.obstacle_checked);
  builder.add_bool(evaluation.obstacles_clear);
  builder.add_double(evaluation.minimum_wall_clearance_m);
  builder.add_double(evaluation.minimum_obstacle_clearance_m);
}

std::uint64_t fingerprint_proof(const RetainedExecutionProof & proof) noexcept
{
  FingerprintBuilder builder;
  add_current(builder, proof.current);
  builder.add_uint64(proof.plan_id);
  add_cursor(builder, proof.cursor);
  builder.add_double(proof.lifted_current_progress_m);
  builder.add_long(proof.lap_offset);
  add_window(builder, proof.window);
  add_segment(builder, proof.measured_to_control_prefix);
  add_segment(builder, proof.control_to_retained_connector);
  builder.add_size(proof.stage_evaluations.size());
  for (const auto & evaluation : proof.stage_evaluations) {
    add_stage_evaluation(builder, evaluation);
  }
  builder.add_bool(proof.physical.checked);
  builder.add_bool(proof.physical.wall_clear);
  builder.add_bool(proof.physical.obstacles_clear);
  builder.add_double(proof.physical.minimum_wall_clearance_m);
  builder.add_double(proof.physical.minimum_obstacle_clearance_m);
  return builder.value();
}

bool segment_identity_matches(
  const RetainedPathSegmentEvaluation & segment,
  const CurrentExecutionProvenance & current,
  const double expected_start_progress_m,
  const double expected_end_progress_m) noexcept
{
  return segment.observation_generation == current.observation_generation &&
         segment.stage_geometry_id == current.stage_geometry_id &&
         segment.target_obstacle_generation ==
         current.target_obstacle_generation &&
         segment.control_pose_id == current.control_pose_id &&
         segment.course_frame_window_id == current.course_frame_window_id &&
         segment.obstacle_tube_id == current.obstacle_tube_id &&
         same_double(segment.start_progress_m, expected_start_progress_m) &&
         same_double(segment.end_progress_m, expected_end_progress_m);
}

bool segment_clear(const RetainedPathSegmentEvaluation & segment) noexcept
{
  return segment.checked && segment.wall_clear && segment.obstacles_clear &&
         nonnegative_finite(segment.minimum_wall_clearance_m) &&
         nonnegative_finite(segment.minimum_obstacle_clearance_m);
}

bool stage_identity_matches(
  const RetainedStageSafetyEvaluation & evaluation,
  const RetainedStageSample & sample,
  const CurrentExecutionProvenance & current) noexcept
{
  return evaluation.control_stage_index == sample.control_stage_index &&
         same_double(evaluation.relative_time_sec, sample.relative_time_sec) &&
         same_double(
    evaluation.segment_duration_sec,
    sample.segment_duration_sec) &&
         same_double(
    evaluation.segment_start_progress_m,
    sample.segment_start_progress_m) &&
         same_double(
    evaluation.absolute_progress_m,
    sample.absolute_progress_m) &&
         evaluation.observation_generation == current.observation_generation &&
         evaluation.stage_geometry_id == current.stage_geometry_id &&
         evaluation.target_obstacle_generation ==
         current.target_obstacle_generation &&
         evaluation.course_frame_window_id == current.course_frame_window_id &&
         evaluation.obstacle_tube_id == current.obstacle_tube_id;
}

} // namespace

const char * to_string(const RetainedExecutionWindowReason reason) noexcept
{
  switch (reason) {
    case RetainedExecutionWindowReason::Accepted:
      return "accepted";
    case RetainedExecutionWindowReason::InvalidPlan:
      return "invalid-plan";
    case RetainedExecutionWindowReason::CursorUnavailable:
      return "cursor-unavailable";
    case RetainedExecutionWindowReason::PlanIdentityMismatch:
      return "plan-identity-mismatch";
    case RetainedExecutionWindowReason::ExecutionWindowMismatch:
      return "execution-window-mismatch";
    case RetainedExecutionWindowReason::InvalidPartialStage:
      return "invalid-partial-stage";
    case RetainedExecutionWindowReason::InvalidProgressEvolution:
      return "invalid-progress-evolution";
  }
  return "unknown";
}

RetainedExecutionWindowResult build_retained_execution_window(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor)
{
  RetainedExecutionWindowResult result;
  if (plan::validate_canonical_execution_plan(execution_plan) !=
    plan::CanonicalExecutionPlanRejectReason::None)
  {
    return result;
  }
  if (!cursor.available) {
    result.reason = RetainedExecutionWindowReason::CursorUnavailable;
    return result;
  }
  if (cursor.plan_id != execution_plan.plan_id) {
    result.reason = RetainedExecutionWindowReason::PlanIdentityMismatch;
    return result;
  }
  const std::size_t first = cursor.first_control_stage_index;
  if (first >= execution_plan.control_stages.size() ||
    first + 1U >= execution_plan.predicted_states.size() ||
    cursor.remaining_control_stage_count !=
    execution_plan.control_stages.size() - first)
  {
    result.reason = RetainedExecutionWindowReason::ExecutionWindowMismatch;
    return result;
  }
  const double first_duration =
    execution_plan.control_stages[first].duration_sec;
  if (!std::isfinite(cursor.stage_elapsed_sec) ||
    cursor.stage_elapsed_sec < 0.0 ||
    cursor.stage_elapsed_sec >= first_duration)
  {
    result.reason = RetainedExecutionWindowReason::InvalidPartialStage;
    return result;
  }
  const double fraction = cursor.stage_elapsed_sec / first_duration;
  const double start_progress =
    execution_plan.predicted_states[first].progress_m;
  const double first_endpoint_progress =
    execution_plan.predicted_states[first + 1U].progress_m;
  if (!std::isfinite(start_progress) ||
    !std::isfinite(first_endpoint_progress) ||
    first_endpoint_progress + kIdentityTolerance < start_progress)
  {
    result.reason = RetainedExecutionWindowReason::InvalidProgressEvolution;
    return result;
  }

  RetainedExecutionWindow window;
  window.plan_id = execution_plan.plan_id;
  window.cursor = cursor;
  const auto & start_state = execution_plan.predicted_states[first];
  const auto & end_state = execution_plan.predicted_states[first + 1U];
  const auto interpolate = [fraction](const double start, const double end) {
      return start + fraction * (end - start);
    };
  window.expected_current_state = plan::CanonicalPredictedState{
    interpolate(start_state.lateral_m, end_state.lateral_m),
    interpolate(start_state.lag_m, end_state.lag_m),
    interpolate(start_state.heading_offset_rad, end_state.heading_offset_rad),
    interpolate(start_state.velocity_mps, end_state.velocity_mps),
    interpolate(start_state.progress_m, end_state.progress_m)};
  window.expected_current_progress_m = window.expected_current_state.progress_m;
  window.samples.reserve(cursor.remaining_control_stage_count);
  double relative_time_sec = 0.0;
  double segment_start_progress_m = window.expected_current_progress_m;
  for (std::size_t index = first; index < execution_plan.control_stages.size();
    ++index)
  {
    const double duration_sec =
      index == first ? execution_plan.control_stages[index].duration_sec -
      cursor.stage_elapsed_sec :
      execution_plan.control_stages[index].duration_sec;
    const auto & endpoint = execution_plan.predicted_states[index + 1U];
    if (!std::isfinite(duration_sec) || duration_sec <= 0.0 ||
      !std::isfinite(endpoint.progress_m) ||
      endpoint.progress_m + kIdentityTolerance < segment_start_progress_m)
    {
      result.reason = RetainedExecutionWindowReason::InvalidProgressEvolution;
      return result;
    }
    relative_time_sec += duration_sec;
    window.samples.push_back(
      RetainedStageSample{
        index, index + 1U, relative_time_sec, duration_sec,
        segment_start_progress_m, endpoint.progress_m, endpoint});
    segment_start_progress_m = endpoint.progress_m;
  }
  result.reason = RetainedExecutionWindowReason::Accepted;
  result.window = std::move(window);
  return result;
}

const char * to_string(const CircularProgressLiftReason reason) noexcept
{
  switch (reason) {
    case CircularProgressLiftReason::Accepted:
      return "accepted";
    case CircularProgressLiftReason::InvalidInput:
      return "invalid-input";
    case CircularProgressLiftReason::AmbiguousBranch:
      return "ambiguous-branch";
    case CircularProgressLiftReason::Discontinuous:
      return "discontinuous";
  }
  return "unknown";
}

CircularProgressLiftResult lift_progress_to_retained_branch(
  const CircularProgressLiftRequest & request) noexcept
{
  CircularProgressLiftResult result;
  if (!std::isfinite(request.measured_progress_m) ||
    !std::isfinite(request.retained_reference_progress_m) ||
    !std::isfinite(request.path_length_m) || request.path_length_m <= 0.0 ||
    !std::isfinite(request.continuity_tolerance_m) ||
    request.continuity_tolerance_m < 0.0)
  {
    return result;
  }
  if (!request.circular) {
    if (std::abs(
        request.measured_progress_m -
        request.retained_reference_progress_m) >
      request.continuity_tolerance_m + kIdentityTolerance)
    {
      result.reason = CircularProgressLiftReason::Discontinuous;
      return result;
    }
    result.reason = CircularProgressLiftReason::Accepted;
    result.lifted_progress_m = request.measured_progress_m;
    return result;
  }
  if (request.measured_progress_m < 0.0 ||
    request.measured_progress_m >= request.path_length_m ||
    request.continuity_tolerance_m >= 0.5 * request.path_length_m)
  {
    result.reason =
      request.continuity_tolerance_m >= 0.5 * request.path_length_m ?
      CircularProgressLiftReason::AmbiguousBranch :
      CircularProgressLiftReason::InvalidInput;
    return result;
  }
  const double raw_offset =
    (request.retained_reference_progress_m - request.measured_progress_m) /
    request.path_length_m;
  if (raw_offset < static_cast<double>(std::numeric_limits<long>::min()) ||
    raw_offset > static_cast<double>(std::numeric_limits<long>::max()))
  {
    return result;
  }
  const long lap_offset = std::lround(raw_offset);
  const double lifted = request.measured_progress_m +
    static_cast<double>(lap_offset) * request.path_length_m;
  if (std::abs(lifted - request.retained_reference_progress_m) >
    request.continuity_tolerance_m + kIdentityTolerance)
  {
    result.reason = CircularProgressLiftReason::Discontinuous;
    return result;
  }
  result.reason = CircularProgressLiftReason::Accepted;
  result.lifted_progress_m = lifted;
  result.lap_offset = lap_offset;
  return result;
}

bool current_execution_provenance_complete(
  const CurrentExecutionProvenance & provenance) noexcept
{
  const bool required_target_present =
    !contract::canonical_normal_intent_requires_target(provenance.intent) ||
    !provenance.target_id.empty();
  const bool target_identity_complete =
    provenance.target_id.empty() ?
    provenance.target_obstacle_generation == 0U :
    provenance.target_obstacle_generation != 0U;
  return provenance.decision_id != 0U &&
         contract::canonical_normal_intent_supported(provenance.intent) &&
         provenance.observation_generation != 0U &&
         provenance.stage_geometry_id != 0U && required_target_present &&
         target_identity_complete &&
         provenance.control_pose_id != 0U &&
         provenance.course_frame_window_id != 0U &&
         provenance.obstacle_tube_id != 0U &&
         nonnegative_finite(provenance.observation_sec) &&
         std::isfinite(provenance.path_length_m) &&
         provenance.path_length_m > 0.0;
}

const char * to_string(const RetainedExecutionProofReason reason) noexcept
{
  switch (reason) {
    case RetainedExecutionProofReason::Accepted:
      return "accepted";
    case RetainedExecutionProofReason::InvalidPlan:
      return "invalid-plan";
    case RetainedExecutionProofReason::CursorUnavailable:
      return "cursor-unavailable";
    case RetainedExecutionProofReason::InvalidCurrentProvenance:
      return "invalid-current-provenance";
    case RetainedExecutionProofReason::IntentMismatch:
      return "intent-mismatch";
    case RetainedExecutionProofReason::IntentGenerationMismatch:
      return "intent-generation-mismatch";
    case RetainedExecutionProofReason::TargetIdentityMismatch:
      return "target-identity-mismatch";
    case RetainedExecutionProofReason::ProgressLiftRejected:
      return "progress-lift-rejected";
    case RetainedExecutionProofReason::PrefixIdentityMismatch:
      return "prefix-identity-mismatch";
    case RetainedExecutionProofReason::DelayPrefixRejected:
      return "delay-prefix-rejected";
    case RetainedExecutionProofReason::ConnectorRejected:
      return "connector-rejected";
    case RetainedExecutionProofReason::StageEvaluationCountMismatch:
      return "stage-evaluation-count-mismatch";
    case RetainedExecutionProofReason::StageEvaluationIdentityMismatch:
      return "stage-evaluation-identity-mismatch";
    case RetainedExecutionProofReason::CourseFrameUnavailable:
      return "course-frame-unavailable";
    case RetainedExecutionProofReason::WallRejected:
      return "wall-rejected";
    case RetainedExecutionProofReason::ObstacleRejected:
      return "obstacle-rejected";
    case RetainedExecutionProofReason::InvalidClearance:
      return "invalid-clearance";
    case RetainedExecutionProofReason::FingerprintMismatch:
      return "fingerprint-mismatch";
  }
  return "unknown";
}

RetainedExecutionProofResult build_retained_execution_proof(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor,
  const RetainedExecutionProofRequest & request)
{
  RetainedExecutionProofResult result;
  const auto window_result =
    build_retained_execution_window(execution_plan, cursor);
  if (!window_result.window.has_value()) {
    result.reason =
      window_result.reason == RetainedExecutionWindowReason::CursorUnavailable ?
      RetainedExecutionProofReason::CursorUnavailable :
      RetainedExecutionProofReason::InvalidPlan;
    return result;
  }
  if (!current_execution_provenance_complete(request.current)) {
    result.reason = RetainedExecutionProofReason::InvalidCurrentProvenance;
    return result;
  }
  if (request.current.intent != execution_plan.problem.intent) {
    result.reason = RetainedExecutionProofReason::IntentMismatch;
    return result;
  }
  if (request.current.intent_generation !=
    execution_plan.problem.intent_generation)
  {
    result.reason = RetainedExecutionProofReason::IntentGenerationMismatch;
    return result;
  }
  if (request.current.target_id != execution_plan.problem.target_id) {
    result.reason = RetainedExecutionProofReason::TargetIdentityMismatch;
    return result;
  }

  const auto lift =
    lift_progress_to_retained_branch(
    CircularProgressLiftRequest{
      request.measured_course_progress_m,
      window_result.window->expected_current_progress_m,
      request.current.path_length_m,
      request.progress_continuity_tolerance_m, request.current.circular});
  if (lift.reason != CircularProgressLiftReason::Accepted) {
    result.reason = RetainedExecutionProofReason::ProgressLiftRejected;
    return result;
  }
  if (!segment_identity_matches(
      request.measured_to_control_prefix,
      request.current, lift.lifted_progress_m, lift.lifted_progress_m) ||
    !segment_identity_matches(
      request.control_to_retained_connector,
      request.current, lift.lifted_progress_m,
      window_result.window->expected_current_progress_m))
  {
    result.reason = RetainedExecutionProofReason::PrefixIdentityMismatch;
    return result;
  }
  if (!segment_clear(request.measured_to_control_prefix)) {
    result.reason = RetainedExecutionProofReason::DelayPrefixRejected;
    return result;
  }
  if (!segment_clear(request.control_to_retained_connector)) {
    result.reason = RetainedExecutionProofReason::ConnectorRejected;
    return result;
  }
  if (request.stage_evaluations.size() !=
    window_result.window->samples.size())
  {
    result.reason = RetainedExecutionProofReason::StageEvaluationCountMismatch;
    return result;
  }

  double minimum_wall_clearance_m =
    std::min(
    request.measured_to_control_prefix.minimum_wall_clearance_m,
    request.control_to_retained_connector.minimum_wall_clearance_m);
  double minimum_obstacle_clearance_m = std::min(
    request.measured_to_control_prefix.minimum_obstacle_clearance_m,
    request.control_to_retained_connector.minimum_obstacle_clearance_m);
  for (std::size_t index = 0U; index < request.stage_evaluations.size();
    ++index)
  {
    const auto & evaluation = request.stage_evaluations[index];
    if (!stage_identity_matches(
        evaluation,
        window_result.window->samples[index],
        request.current))
    {
      result.reason =
        RetainedExecutionProofReason::StageEvaluationIdentityMismatch;
      return result;
    }
    if (!evaluation.course_frame_available) {
      result.reason = RetainedExecutionProofReason::CourseFrameUnavailable;
      return result;
    }
    if (!evaluation.wall_checked || !evaluation.wall_clear) {
      result.reason = RetainedExecutionProofReason::WallRejected;
      return result;
    }
    if (!evaluation.obstacle_checked || !evaluation.obstacles_clear) {
      result.reason = RetainedExecutionProofReason::ObstacleRejected;
      return result;
    }
    if (!nonnegative_finite(evaluation.minimum_wall_clearance_m) ||
      !nonnegative_finite(evaluation.minimum_obstacle_clearance_m))
    {
      result.reason = RetainedExecutionProofReason::InvalidClearance;
      return result;
    }
    minimum_wall_clearance_m =
      std::min(minimum_wall_clearance_m, evaluation.minimum_wall_clearance_m);
    minimum_obstacle_clearance_m = std::min(
      minimum_obstacle_clearance_m, evaluation.minimum_obstacle_clearance_m);
  }

  RetainedExecutionProof proof;
  proof.current = request.current;
  proof.plan_id = execution_plan.plan_id;
  proof.cursor = cursor;
  proof.lifted_current_progress_m = lift.lifted_progress_m;
  proof.lap_offset = lift.lap_offset;
  proof.window = window_result.window.value();
  proof.measured_to_control_prefix = request.measured_to_control_prefix;
  proof.control_to_retained_connector = request.control_to_retained_connector;
  proof.stage_evaluations = request.stage_evaluations;
  proof.physical.checked = true;
  proof.physical.wall_clear = true;
  proof.physical.obstacles_clear = true;
  proof.physical.minimum_wall_clearance_m = minimum_wall_clearance_m;
  proof.physical.minimum_obstacle_clearance_m = minimum_obstacle_clearance_m;
  proof.proof_fingerprint = fingerprint_proof(proof);
  result.reason = RetainedExecutionProofReason::Accepted;
  result.proof = std::move(proof);
  return result;
}

RetainedExecutionProofReason validate_retained_execution_proof(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor,
  const CurrentExecutionProvenance & current,
  const RetainedExecutionProof & proof)
{
  if (plan::validate_canonical_execution_plan(execution_plan) !=
    plan::CanonicalExecutionPlanRejectReason::None)
  {
    return RetainedExecutionProofReason::InvalidPlan;
  }
  if (!cursor.available) {
    return RetainedExecutionProofReason::CursorUnavailable;
  }
  if (!current_execution_provenance_complete(current) ||
    !same_current(current, proof.current))
  {
    return RetainedExecutionProofReason::FingerprintMismatch;
  }
  if (proof.plan_id != execution_plan.plan_id ||
    !same_cursor(cursor, proof.cursor))
  {
    return RetainedExecutionProofReason::FingerprintMismatch;
  }
  const auto expected_window =
    build_retained_execution_window(execution_plan, cursor);
  if (!expected_window.window.has_value() ||
    !same_window(expected_window.window.value(), proof.window))
  {
    return RetainedExecutionProofReason::FingerprintMismatch;
  }
  if (proof.proof_fingerprint == 0U ||
    proof.proof_fingerprint != fingerprint_proof(proof))
  {
    return RetainedExecutionProofReason::FingerprintMismatch;
  }
  if (!proof.physical.checked || !proof.physical.wall_clear ||
    !proof.physical.obstacles_clear)
  {
    return RetainedExecutionProofReason::WallRejected;
  }
  return RetainedExecutionProofReason::Accepted;
}

const char * to_string(const RetainedCandidateBuildReason reason) noexcept
{
  switch (reason) {
    case RetainedCandidateBuildReason::Accepted:
      return "accepted";
    case RetainedCandidateBuildReason::InvalidPlan:
      return "invalid-plan";
    case RetainedCandidateBuildReason::CursorUnavailable:
      return "cursor-unavailable";
    case RetainedCandidateBuildReason::ProofRejected:
      return "proof-rejected";
  }
  return "unknown";
}

RetainedCandidateBuildResult build_canonical_retained_candidate(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor,
  const CurrentExecutionProvenance & current,
  const RetainedExecutionProof & proof)
{
  RetainedCandidateBuildResult result;
  if (plan::validate_canonical_execution_plan(execution_plan) !=
    plan::CanonicalExecutionPlanRejectReason::None)
  {
    return result;
  }
  if (!cursor.available) {
    result.reason = RetainedCandidateBuildReason::CursorUnavailable;
    result.proof_reason = RetainedExecutionProofReason::CursorUnavailable;
    return result;
  }
  result.proof_reason =
    validate_retained_execution_proof(execution_plan, cursor, current, proof);
  if (result.proof_reason != RetainedExecutionProofReason::Accepted) {
    result.reason = RetainedCandidateBuildReason::ProofRejected;
    return result;
  }

  contract::CanonicalNormalCandidate candidate;
  candidate.problem = execution_plan.problem;
  candidate.solution = execution_plan.solution;
  candidate.executable_control_stage_count =
    cursor.remaining_control_stage_count;
  candidate.execution_plan_id = execution_plan.plan_id;
  candidate.execution_certificate_decision_id = current.decision_id;
  candidate.execution_first_control_stage_index =
    cursor.first_control_stage_index;
  candidate.execution_physical = proof.physical;
  result.reason = RetainedCandidateBuildReason::Accepted;
  result.candidate = std::move(candidate);
  return result;
}

} // namespace multi_purpose_mpc_ros::canonical_retained_revalidation
