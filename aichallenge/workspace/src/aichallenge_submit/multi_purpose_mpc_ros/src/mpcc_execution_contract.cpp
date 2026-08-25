#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <type_traits>

namespace multi_purpose_mpc_ros::mpcc_execution_contract
{
namespace
{

constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
constexpr double kKinematicEpsilon = 1e-12;

double wrap_to_pi(const double angle) noexcept
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

class FingerprintBuilder
{
public:
  void append_byte(const std::uint8_t value) noexcept
  {
    value_ ^= value;
    value_ *= kFnvPrime;
  }

  void append_u64(const std::uint64_t value) noexcept
  {
    for (unsigned int shift = 0U; shift < 64U; shift += 8U) {
      append_byte(static_cast<std::uint8_t>((value >> shift) & 0xffU));
    }
  }

  void append_i64(const std::int64_t value) noexcept
  {
    append_u64(static_cast<std::uint64_t>(value));
  }

  void append_bool(const bool value) noexcept
  {
    append_byte(value ? 1U : 0U);
  }

  void append_double(double value) noexcept
  {
    if (value == 0.0) {
      value = 0.0;
    }
    std::uint64_t bits{};
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    append_u64(bits);
  }

  void append_string(const std::string & value) noexcept
  {
    append_u64(static_cast<std::uint64_t>(value.size()));
    for (const unsigned char character : value) {
      append_byte(character);
    }
  }

  template<typename EnumT>
  void append_enum(const EnumT value) noexcept
  {
    static_assert(std::is_enum_v<EnumT>);
    append_i64(static_cast<std::int64_t>(value));
  }

  std::uint64_t finish() const noexcept
  {
    return value_ == 0U ? 1U : value_;
  }

private:
  std::uint64_t value_{kFnvOffset};
};

bool schemas_complete(const MpccProblemContext & context) noexcept
{
  return
    !context.state_schema_id.empty() && !context.input_schema_id.empty() &&
    !context.bounds_schema_id.empty() && !context.cost_schema_id.empty();
}

std::string fingerprint_text(const std::uint64_t value)
{
  if (value == 0U) {
    return "none";
  }
  std::ostringstream stream;
  stream << "0x" << std::hex << std::setw(16) << std::setfill('0') << value;
  return stream.str();
}

}  // namespace

std::optional<FrenetPose> project_planar_pose_to_frenet(
  const PlanarPose & pose, const PlanarPose & course_frame) noexcept
{
  if (
    !std::isfinite(pose.x_m) || !std::isfinite(pose.y_m) ||
    !std::isfinite(pose.yaw_rad) || !std::isfinite(course_frame.x_m) ||
    !std::isfinite(course_frame.y_m) || !std::isfinite(course_frame.yaw_rad))
  {
    return std::nullopt;
  }
  const double delta_x_m = pose.x_m - course_frame.x_m;
  const double delta_y_m = pose.y_m - course_frame.y_m;
  const double cos_heading = std::cos(course_frame.yaw_rad);
  const double sin_heading = std::sin(course_frame.yaw_rad);
  FrenetPose result;
  result.lateral_m = cos_heading * delta_y_m - sin_heading * delta_x_m;
  result.lag_m = cos_heading * delta_x_m + sin_heading * delta_y_m;
  result.heading_offset_rad = wrap_to_pi(pose.yaw_rad - course_frame.yaw_rad);
  if (
    !std::isfinite(result.lateral_m) || !std::isfinite(result.lag_m) ||
    !std::isfinite(result.heading_offset_rad))
  {
    return std::nullopt;
  }
  return result;
}

std::optional<PlanarPose> reconstruct_planar_pose_from_frenet(
  const PlanarPose & course_frame, const FrenetPose & state) noexcept
{
  if (
    !std::isfinite(course_frame.x_m) || !std::isfinite(course_frame.y_m) ||
    !std::isfinite(course_frame.yaw_rad) || !std::isfinite(state.lateral_m) ||
    !std::isfinite(state.lag_m) || !std::isfinite(state.heading_offset_rad))
  {
    return std::nullopt;
  }
  const double cos_heading = std::cos(course_frame.yaw_rad);
  const double sin_heading = std::sin(course_frame.yaw_rad);
  PlanarPose result;
  result.x_m = course_frame.x_m + state.lag_m * cos_heading -
    state.lateral_m * sin_heading;
  result.y_m = course_frame.y_m + state.lag_m * sin_heading +
    state.lateral_m * cos_heading;
  result.yaw_rad = wrap_to_pi(
    course_frame.yaw_rad + state.heading_offset_rad);
  if (
    !std::isfinite(result.x_m) || !std::isfinite(result.y_m) ||
    !std::isfinite(result.yaw_rad))
  {
    return std::nullopt;
  }
  return result;
}

std::optional<FirstStageKinematicResult> integrate_first_stage_constant_curvature(
  const FirstStageKinematicRequest & request) noexcept
{
  if (
    !std::isfinite(request.initial_pose.x_m) ||
    !std::isfinite(request.initial_pose.y_m) ||
    !std::isfinite(request.initial_pose.yaw_rad) ||
    !std::isfinite(request.initial_speed_mps) || request.initial_speed_mps < 0.0 ||
    !std::isfinite(request.acceleration_mps2) ||
    !std::isfinite(request.curvature_radpm) ||
    !std::isfinite(request.stage_dt_sec) || request.stage_dt_sec <= 0.0 ||
    !std::isfinite(request.elapsed_sec) || request.elapsed_sec < 0.0 ||
    request.elapsed_sec > request.stage_dt_sec + kKinematicEpsilon)
  {
    return std::nullopt;
  }

  double active_motion_sec = request.elapsed_sec;
  if (request.acceleration_mps2 < -kKinematicEpsilon) {
    const double stop_time_sec =
      request.initial_speed_mps / -request.acceleration_mps2;
    active_motion_sec = std::min(active_motion_sec, stop_time_sec);
  }
  const double travel_distance_m = std::max(
    0.0,
    request.initial_speed_mps * active_motion_sec +
    0.5 * request.acceleration_mps2 * active_motion_sec * active_motion_sec);
  if (!std::isfinite(travel_distance_m)) {
    return std::nullopt;
  }

  FirstStageKinematicResult result;
  result.pose = request.initial_pose;
  result.travel_distance_m = travel_distance_m;
  result.active_motion_sec = active_motion_sec;
  if (std::abs(request.curvature_radpm) <= kKinematicEpsilon) {
    result.pose.x_m += travel_distance_m * std::cos(request.initial_pose.yaw_rad);
    result.pose.y_m += travel_distance_m * std::sin(request.initial_pose.yaw_rad);
    return result;
  }

  const double final_yaw =
    request.initial_pose.yaw_rad + travel_distance_m * request.curvature_radpm;
  result.pose.x_m +=
    (std::sin(final_yaw) - std::sin(request.initial_pose.yaw_rad)) /
    request.curvature_radpm;
  result.pose.y_m +=
    (-std::cos(final_yaw) + std::cos(request.initial_pose.yaw_rad)) /
    request.curvature_radpm;
  result.pose.yaw_rad = wrap_to_pi(final_yaw);
  if (
    !std::isfinite(result.pose.x_m) || !std::isfinite(result.pose.y_m) ||
    !std::isfinite(result.pose.yaw_rad))
  {
    return std::nullopt;
  }
  return result;
}

const char * to_string(const ControlIntent intent) noexcept
{
  switch (intent) {
    case ControlIntent::Unknown: return "unknown";
    case ControlIntent::Track: return "track";
    case ControlIntent::Cruise: return "cruise";
    case ControlIntent::Follow: return "follow";
    case ControlIntent::Hold: return "hold";
    case ControlIntent::Stop: return "stop";
    case ControlIntent::ShiftOut: return "shiftout";
    case ControlIntent::Pass: return "pass";
    case ControlIntent::Return: return "return";
    case ControlIntent::Rejoin: return "rejoin";
  }
  return "unknown";
}

const char * to_string(const Formulation formulation) noexcept
{
  switch (formulation) {
    case Formulation::Unresolved: return "unresolved";
    case Formulation::LegacySpatialMpc3State: return "legacy-spatial-mpc-3state";
    case Formulation::ProgressContouring3State: return "progress-contouring-3state";
    case Formulation::VelocityProgress5State: return "velocity-progress-5state";
    case Formulation::VelocitySteeringProgress6State:
      return "velocity-steering-progress-6state";
    case Formulation::SolverDerivedBypass: return "solver-derived-bypass";
  }
  return "unknown";
}

bool canonical_normal_intent_supported(const ControlIntent intent) noexcept
{
  return
    intent == ControlIntent::Track || intent == ControlIntent::Cruise ||
    intent == ControlIntent::Follow || intent == ControlIntent::ShiftOut ||
    intent == ControlIntent::Pass || intent == ControlIntent::Return ||
    intent == ControlIntent::Rejoin;
}

bool canonical_normal_intent_requires_target(const ControlIntent intent) noexcept
{
  return
    intent == ControlIntent::Follow || intent == ControlIntent::ShiftOut ||
    intent == ControlIntent::Pass || intent == ControlIntent::Return;
}

bool canonical_normal_intent_requires_execution_side(
  const ControlIntent intent) noexcept
{
  return
    intent == ControlIntent::ShiftOut || intent == ControlIntent::Pass ||
    intent == ControlIntent::Return;
}

bool canonical_normal_formulation_supported(
  const Formulation formulation) noexcept
{
  return
    formulation == Formulation::VelocityProgress5State ||
    formulation == Formulation::VelocitySteeringProgress6State;
}

std::uint64_t fingerprint_stage_geometry(
  const int tracking_waypoint, const bool circular,
  const std::vector<StageGeometryIdentity> & stages) noexcept
{
  if (stages.empty()) {
    return 0U;
  }
  FingerprintBuilder builder;
  builder.append_string("mpcc-stage-geometry-v1");
  builder.append_i64(tracking_waypoint);
  builder.append_bool(circular);
  builder.append_u64(static_cast<std::uint64_t>(stages.size()));
  for (const auto & stage : stages) {
    if (
      !std::isfinite(stage.transition_distance_m) ||
      !std::isfinite(stage.cumulative_distance_m))
    {
      return 0U;
    }
    builder.append_i64(stage.transition_from_waypoint);
    builder.append_i64(stage.state_waypoint);
    builder.append_double(stage.transition_distance_m);
    builder.append_double(stage.cumulative_distance_m);
  }
  return builder.finish();
}

std::optional<EffectiveStageGeometry> resolve_effective_stage_geometry(
  const int tracking_waypoint, const bool circular,
  const std::vector<StageGeometryIdentity> & raw_stages,
  const std::vector<double> & effective_transition_distances_m) noexcept
{
  if (
    tracking_waypoint < 0 || raw_stages.empty() ||
    raw_stages.size() != effective_transition_distances_m.size() ||
    raw_stages.front().transition_from_waypoint != tracking_waypoint)
  {
    return std::nullopt;
  }

  EffectiveStageGeometry result;
  result.tracking_waypoint = tracking_waypoint;
  result.circular = circular;
  result.stages.reserve(raw_stages.size());
  double cumulative_distance_m = 0.0;
  for (std::size_t index = 0U; index < raw_stages.size(); ++index) {
    const auto & raw = raw_stages[index];
    const double effective_distance_m = effective_transition_distances_m[index];
    if (
      raw.transition_from_waypoint < 0 || raw.state_waypoint < 0 ||
      !std::isfinite(raw.transition_distance_m) || raw.transition_distance_m < 0.0 ||
      !std::isfinite(raw.cumulative_distance_m) || raw.cumulative_distance_m < 0.0 ||
      !std::isfinite(effective_distance_m) || effective_distance_m <= 0.0 ||
      (index > 0U &&
      raw_stages[index - 1U].state_waypoint != raw.transition_from_waypoint))
    {
      return std::nullopt;
    }
    cumulative_distance_m += effective_distance_m;
    if (!std::isfinite(cumulative_distance_m)) {
      return std::nullopt;
    }
    result.stages.push_back(StageGeometryIdentity{
      raw.transition_from_waypoint, raw.state_waypoint,
      effective_distance_m, cumulative_distance_m});
  }
  result.fingerprint = fingerprint_stage_geometry(
    tracking_waypoint, circular, result.stages);
  if (result.fingerprint == 0U) {
    return std::nullopt;
  }
  return result;
}

const char * physical_wall_certificate_reason_name(
  const PhysicalWallCertificateReason reason) noexcept
{
  switch (reason) {
    case PhysicalWallCertificateReason::NotEvaluated:
      return "not-evaluated";
    case PhysicalWallCertificateReason::Accepted:
      return "accepted";
    case PhysicalWallCertificateReason::InvalidInput:
      return "invalid-input";
    case PhysicalWallCertificateReason::LateralBoundViolation:
      return "lateral-bound-violation";
    case PhysicalWallCertificateReason::HeadingUnavailable:
      return "heading-unavailable";
    case PhysicalWallCertificateReason::WallSampleUnavailable:
      return "wall-sample-unavailable";
    case PhysicalWallCertificateReason::HardWallContact:
      return "hard-wall-contact";
    case PhysicalWallCertificateReason::CurrentPoseWallSampleUnavailable:
      return "current-pose-wall-sample-unavailable";
    case PhysicalWallCertificateReason::CurrentPoseHardWallContact:
      return "current-pose-hard-wall-contact";
    case PhysicalWallCertificateReason::CourseFrameUnavailable:
      return "course-frame-unavailable";
    case PhysicalWallCertificateReason::SweptPathViolation:
      return "swept-path-violation";
  }
  return "unknown";
}

PhysicalWallPathFailureLocation resolve_swept_path_failure_origin(
  const std::size_t rejected_path_index,
  const std::size_t horizon_steps) noexcept
{
  if (rejected_path_index == 0U) {
    return {PhysicalWallPathFailureOrigin::CurrentPose, -1};
  }
  if (
    rejected_path_index <= horizon_steps &&
    rejected_path_index - 1U <=
    static_cast<std::size_t>(std::numeric_limits<int>::max()))
  {
    return {
      PhysicalWallPathFailureOrigin::HorizonStage,
      static_cast<int>(rejected_path_index - 1U)};
  }
  return {};
}

std::string format_physical_wall_certificate_diagnostic(
  const PhysicalWallCertificateDiagnostic & diagnostic)
{
  std::ostringstream output;
  output << std::fixed << std::setprecision(3)
         << "reason=" << physical_wall_certificate_reason_name(diagnostic.reason)
         << ", stage=" << diagnostic.stage_index
         << ", wp=" << diagnostic.waypoint_id;
  if (std::isfinite(diagnostic.path_distance_m)) {
    output << ", distance=" << diagnostic.path_distance_m << "m";
  }
  if (std::isfinite(diagnostic.lateral_m)) {
    output << ", lateral=" << diagnostic.lateral_m << "m";
  }
  if (std::isfinite(diagnostic.lag_m)) {
    output << ", lag=" << diagnostic.lag_m << "m";
  }
  if (
    std::isfinite(diagnostic.lower_bound_m) &&
    std::isfinite(diagnostic.upper_bound_m))
  {
    output << ", bounds=[" << diagnostic.lower_bound_m << ","
           << diagnostic.upper_bound_m << "]m";
  }
  if (std::isfinite(diagnostic.bound_reserve_m)) {
    output << ", reserve=" << diagnostic.bound_reserve_m << "m";
  }
  if (std::isfinite(diagnostic.heading_offset_rad)) {
    output << ", heading_offset=" << diagnostic.heading_offset_rad << "rad";
  }
  if (
    std::isfinite(diagnostic.solved_progress_m) &&
    std::isfinite(diagnostic.reference_progress_m))
  {
    output << ", progress=" << diagnostic.solved_progress_m << "/"
           << diagnostic.reference_progress_m << "m";
  }
  if (std::isfinite(diagnostic.progress_delta_m)) {
    output << ", progress_delta=" << diagnostic.progress_delta_m << "m";
  }
  if (
    std::isfinite(diagnostic.pose_x_m) &&
    std::isfinite(diagnostic.pose_y_m) &&
    std::isfinite(diagnostic.pose_yaw_rad))
  {
    output << ", pose=(" << diagnostic.pose_x_m << ","
           << diagnostic.pose_y_m << "," << diagnostic.pose_yaw_rad << ")";
  }
  output << ", out_of_map=" << (diagnostic.out_of_map ? 1 : 0)
         << ", contacts=" << diagnostic.contact_cell_count;
  if (
    diagnostic.swept_rejected_path_index !=
    std::numeric_limits<std::size_t>::max())
  {
    output << ", swept_index=" << diagnostic.swept_rejected_path_index;
  }
  if (diagnostic.swept_checked_pose_count > 0U) {
    output << ", swept_checked=" << diagnostic.swept_checked_pose_count;
  }
  if (std::isfinite(diagnostic.swept_rejected_segment_ratio)) {
    output << ", swept_substep=" << diagnostic.swept_rejected_substep << "/"
           << diagnostic.swept_rejected_subdivision_count
           << ", swept_ratio=" << diagnostic.swept_rejected_segment_ratio;
  }
  return output.str();
}

std::uint64_t problem_context_fingerprint(
  const MpccProblemContext & context) noexcept
{
  FingerprintBuilder builder;
  builder.append_string("mpcc-problem-context-v1");
  builder.append_u64(context.decision_id);
  builder.append_enum(context.intent);
  builder.append_u64(context.intent_generation);
  builder.append_u64(context.observation_generation);
  builder.append_u64(context.stage_geometry_id);
  builder.append_u64(context.target_obstacle_generation);
  builder.append_string(context.target_id);
  builder.append_i64(context.execution_side_sign);
  builder.append_u64(static_cast<std::uint64_t>(context.horizon_steps));
  builder.append_enum(context.formulation);
  builder.append_string(context.state_schema_id);
  builder.append_string(context.input_schema_id);
  builder.append_string(context.bounds_schema_id);
  builder.append_string(context.cost_schema_id);
  return builder.finish();
}

MpccProblemContext seal_problem_context(MpccProblemContext context) noexcept
{
  context.fingerprint = problem_context_fingerprint(context);
  return context;
}

bool problem_context_complete(const MpccProblemContext & context) noexcept
{
  const bool required_target_present =
    !canonical_normal_intent_requires_target(context.intent) ||
    !context.target_id.empty();
  const bool target_generation_complete =
    context.target_id.empty() ||
    (context.observation_generation > 0U && context.target_obstacle_generation > 0U);
  const bool execution_side_complete =
    canonical_normal_intent_requires_execution_side(context.intent) ?
    (context.execution_side_sign == -1 || context.execution_side_sign == 1) :
    context.execution_side_sign == 0;
  return
    context.decision_id > 0U && context.intent != ControlIntent::Unknown &&
    context.stage_geometry_id > 0U && context.horizon_steps > 0U &&
    context.formulation != Formulation::Unresolved && schemas_complete(context) &&
    required_target_present && target_generation_complete &&
    execution_side_complete && context.fingerprint > 0U &&
    context.fingerprint == problem_context_fingerprint(context);
}

bool solution_certified(const CertifiedMpccSolution & solution) noexcept
{
  return
    solution.solution_id > 0U && solution.problem_fingerprint > 0U &&
    solution.formulation != Formulation::Unresolved && solution.solved &&
    solution.finite && solution.constraints_satisfied &&
    std::isfinite(solution.maximum_constraint_violation) &&
    solution.maximum_constraint_violation >= 0.0 &&
    solution.physical.checked && solution.physical.wall_clear &&
    solution.physical.obstacles_clear && solution.prediction_stage_count > 0U &&
    std::isfinite(solution.valid_until_sec);
}

const char * to_string(
  const CanonicalNormalCandidateRejectReason reason) noexcept
{
  switch (reason) {
    case CanonicalNormalCandidateRejectReason::NotEvaluated:
      return "not-evaluated";
    case CanonicalNormalCandidateRejectReason::None:
      return "none";
    case CanonicalNormalCandidateRejectReason::MissingIdentity:
      return "missing-identity";
    case CanonicalNormalCandidateRejectReason::IncompleteProblem:
      return "incomplete-problem";
    case CanonicalNormalCandidateRejectReason::UnsupportedIntent:
      return "unsupported-intent";
    case CanonicalNormalCandidateRejectReason::IntentMismatch:
      return "intent-mismatch";
    case CanonicalNormalCandidateRejectReason::NoncanonicalFormulation:
      return "noncanonical-formulation";
    case CanonicalNormalCandidateRejectReason::IdentityMismatch:
      return "identity-mismatch";
    case CanonicalNormalCandidateRejectReason::NotCertified:
      return "not-certified";
    case CanonicalNormalCandidateRejectReason::MissingExecutionPlan:
      return "missing-execution-plan";
    case CanonicalNormalCandidateRejectReason::NoExecutableControl:
      return "no-executable-control";
    case CanonicalNormalCandidateRejectReason::InvalidExecutableHorizon:
      return "invalid-executable-horizon";
    case CanonicalNormalCandidateRejectReason::Expired:
      return "expired";
    case CanonicalNormalCandidateRejectReason::DecisionMismatch:
      return "decision-mismatch";
    case CanonicalNormalCandidateRejectReason::ExecutionCertificateDecisionMismatch:
      return "execution-certificate-decision-mismatch";
    case CanonicalNormalCandidateRejectReason::ExecutionCertificateNotCertified:
      return "execution-certificate-not-certified";
  }
  return "unknown";
}

const char * to_string(const CanonicalNormalAuthoritySource source) noexcept
{
  switch (source) {
    case CanonicalNormalAuthoritySource::FreshCertified:
      return "fresh-certified";
    case CanonicalNormalAuthoritySource::RetainedCertified:
      return "retained-certified";
    case CanonicalNormalAuthoritySource::EmergencyStop:
      return "emergency-stop";
  }
  return "unknown";
}

const char * to_string(const CanonicalNormalAuthorityReason reason) noexcept
{
  switch (reason) {
    case CanonicalNormalAuthorityReason::FreshCertified:
      return "fresh-certified";
    case CanonicalNormalAuthorityReason::RetainedCertified:
      return "retained-certified";
    case CanonicalNormalAuthorityReason::NoCanonicalCandidate:
      return "no-canonical-candidate";
    case CanonicalNormalAuthorityReason::InvalidRequest:
      return "invalid-request";
  }
  return "unknown";
}

namespace
{

CanonicalNormalCandidateRejectReason qualify_canonical_normal_candidate(
  const CanonicalNormalCandidate & candidate, const double now_sec,
  const std::uint64_t current_decision_id,
  const ControlIntent current_intent,
  const bool require_current_decision) noexcept
{
  if (!candidate.problem.has_value() || !candidate.solution.has_value()) {
    return CanonicalNormalCandidateRejectReason::MissingIdentity;
  }
  const auto & problem = candidate.problem.value();
  const auto & solution = candidate.solution.value();
  if (!problem_context_complete(problem)) {
    return CanonicalNormalCandidateRejectReason::IncompleteProblem;
  }
  if (!canonical_normal_intent_supported(problem.intent)) {
    return CanonicalNormalCandidateRejectReason::UnsupportedIntent;
  }
  if (problem.intent != current_intent) {
    return CanonicalNormalCandidateRejectReason::IntentMismatch;
  }
  if (
    !canonical_normal_formulation_supported(problem.formulation) ||
    !canonical_normal_formulation_supported(solution.formulation))
  {
    return CanonicalNormalCandidateRejectReason::NoncanonicalFormulation;
  }
  if (solution.problem_fingerprint != problem.fingerprint) {
    return CanonicalNormalCandidateRejectReason::IdentityMismatch;
  }
  if (!solution_certified(solution)) {
    return CanonicalNormalCandidateRejectReason::NotCertified;
  }
  if (candidate.execution_plan_id == 0U) {
    return CanonicalNormalCandidateRejectReason::MissingExecutionPlan;
  }
  if (candidate.executable_control_stage_count == 0U) {
    return CanonicalNormalCandidateRejectReason::NoExecutableControl;
  }
  if (
    candidate.execution_first_control_stage_index >=
    solution.prediction_stage_count ||
    candidate.executable_control_stage_count >
    solution.prediction_stage_count -
    candidate.execution_first_control_stage_index ||
    candidate.executable_control_stage_count !=
    solution.prediction_stage_count -
    candidate.execution_first_control_stage_index)
  {
    return CanonicalNormalCandidateRejectReason::InvalidExecutableHorizon;
  }
  if (now_sec > solution.valid_until_sec) {
    return CanonicalNormalCandidateRejectReason::Expired;
  }
  if (require_current_decision && problem.decision_id != current_decision_id) {
    return CanonicalNormalCandidateRejectReason::DecisionMismatch;
  }
  if (
    candidate.execution_certificate_decision_id != current_decision_id)
  {
    return CanonicalNormalCandidateRejectReason::
      ExecutionCertificateDecisionMismatch;
  }
  if (
    !candidate.execution_physical.checked ||
    !candidate.execution_physical.wall_clear ||
    !candidate.execution_physical.obstacles_clear)
  {
    return CanonicalNormalCandidateRejectReason::
      ExecutionCertificateNotCertified;
  }
  return CanonicalNormalCandidateRejectReason::None;
}

}  // namespace

CanonicalNormalAuthorityResolution resolve_canonical_normal_authority(
  const CanonicalNormalAuthorityRequest & request) noexcept
{
  CanonicalNormalAuthorityResolution resolution;
  if (
    request.current_decision_id == 0U || !std::isfinite(request.now_sec) ||
    request.now_sec < 0.0 ||
    !canonical_normal_intent_supported(request.current_intent))
  {
    resolution.reason = CanonicalNormalAuthorityReason::InvalidRequest;
    return resolution;
  }

  resolution.fresh_reject_reason = qualify_canonical_normal_candidate(
    request.fresh, request.now_sec, request.current_decision_id,
    request.current_intent, true);
  if (
    resolution.fresh_reject_reason ==
    CanonicalNormalCandidateRejectReason::None)
  {
    resolution.source = CanonicalNormalAuthoritySource::FreshCertified;
    resolution.reason = CanonicalNormalAuthorityReason::FreshCertified;
    resolution.problem = request.fresh.problem;
    resolution.solution = request.fresh.solution;
    resolution.executable_control_stage_count =
      request.fresh.executable_control_stage_count;
    resolution.execution_plan_id = request.fresh.execution_plan_id;
    resolution.execution_certificate_decision_id =
      request.fresh.execution_certificate_decision_id;
    resolution.execution_first_control_stage_index =
      request.fresh.execution_first_control_stage_index;
    return resolution;
  }

  resolution.retained_reject_reason = qualify_canonical_normal_candidate(
    request.retained, request.now_sec, request.current_decision_id,
    request.current_intent, false);
  if (
    resolution.retained_reject_reason ==
    CanonicalNormalCandidateRejectReason::None)
  {
    resolution.source = CanonicalNormalAuthoritySource::RetainedCertified;
    resolution.reason = CanonicalNormalAuthorityReason::RetainedCertified;
    resolution.problem = request.retained.problem;
    resolution.solution = request.retained.solution;
    resolution.executable_control_stage_count =
      request.retained.executable_control_stage_count;
    resolution.execution_plan_id = request.retained.execution_plan_id;
    resolution.execution_certificate_decision_id =
      request.retained.execution_certificate_decision_id;
    resolution.execution_first_control_stage_index =
      request.retained.execution_first_control_stage_index;
    resolution.retained_solution = true;
    return resolution;
  }

  resolution.source = CanonicalNormalAuthoritySource::EmergencyStop;
  resolution.reason = CanonicalNormalAuthorityReason::NoCanonicalCandidate;
  return resolution;
}

const char * to_string(const CanonicalNormalCommandReason reason) noexcept
{
  switch (reason) {
    case CanonicalNormalCommandReason::Available: return "available";
    case CanonicalNormalCommandReason::EmergencyAuthority:
      return "emergency-authority";
    case CanonicalNormalCommandReason::IncompleteAuthorityIdentity:
      return "incomplete-authority-identity";
    case CanonicalNormalCommandReason::InvalidActuation:
      return "invalid-actuation";
  }
  return "unknown";
}

CanonicalNormalCommandResult build_canonical_normal_command(
  const CanonicalNormalAuthorityResolution & authority,
  const CanonicalActuation & actuation) noexcept
{
  CanonicalNormalCommandResult result;
  if (authority.source == CanonicalNormalAuthoritySource::EmergencyStop) {
    result.reason = CanonicalNormalCommandReason::EmergencyAuthority;
    return result;
  }
  if (
    !authority.problem.has_value() || !authority.solution.has_value() ||
    !problem_context_complete(authority.problem.value()) ||
    !solution_certified(authority.solution.value()) ||
    authority.execution_plan_id == 0U ||
    authority.execution_certificate_decision_id == 0U ||
    !canonical_normal_formulation_supported(authority.problem->formulation) ||
    !canonical_normal_formulation_supported(authority.solution->formulation) ||
    authority.solution->problem_fingerprint != authority.problem->fingerprint)
  {
    result.reason = CanonicalNormalCommandReason::IncompleteAuthorityIdentity;
    return result;
  }
  if (
    !std::isfinite(actuation.predicted_speed_mps) ||
    actuation.predicted_speed_mps < 0.0 ||
    !std::isfinite(actuation.acceleration_mps2) ||
    !std::isfinite(actuation.curvature_radpm) ||
    !std::isfinite(actuation.steering_tire_angle_rad) ||
    !std::isfinite(actuation.virtual_progress_speed_mps) ||
    actuation.virtual_progress_speed_mps < 0.0)
  {
    result.reason = CanonicalNormalCommandReason::InvalidActuation;
    return result;
  }

  result.command = CanonicalNormalCommand{
    authority.execution_certificate_decision_id,
    authority.execution_plan_id,
    authority.execution_certificate_decision_id,
    authority.problem->fingerprint,
    authority.solution->solution_id,
    authority.source,
    authority.problem->intent,
    authority.problem->formulation,
    authority.retained_solution,
    actuation.predicted_speed_mps,
    actuation.acceleration_mps2,
    actuation.curvature_radpm,
    actuation.steering_tire_angle_rad,
    actuation.virtual_progress_speed_mps};
  result.reason = CanonicalNormalCommandReason::Available;
  return result;
}

bool canonical_normal_command_matches_actuation(
  const CanonicalNormalCommand & command, const double target_speed_mps,
  const double acceleration_mps2,
  const double steering_tire_angle_rad) noexcept
{
  return
    command.source != CanonicalNormalAuthoritySource::EmergencyStop &&
    std::isfinite(target_speed_mps) &&
    std::isfinite(acceleration_mps2) &&
    std::isfinite(steering_tire_angle_rad) &&
    target_speed_mps == command.predicted_speed_mps &&
    acceleration_mps2 == command.acceleration_mps2 &&
    steering_tire_angle_rad == command.steering_tire_angle_rad;
}

std::optional<double> resolve_published_steering_tire_angle(
  const double model_steering_tire_angle_rad,
  const double legacy_actuator_gain,
  const bool canonical_normal_authority) noexcept
{
  if (
    !std::isfinite(model_steering_tire_angle_rad) ||
    !std::isfinite(legacy_actuator_gain) || legacy_actuator_gain <= 0.0)
  {
    return std::nullopt;
  }
  const double published = canonical_normal_authority ?
    model_steering_tire_angle_rad :
    model_steering_tire_angle_rad * legacy_actuator_gain;
  if (!std::isfinite(published)) {
    return std::nullopt;
  }
  return published;
}

bool canonical_normal_uses_physical_steering(
  const bool canonical_normal_authority,
  const bool canonical_emergency_stop,
  const bool recovery_override) noexcept
{
  return
    !recovery_override &&
    (canonical_normal_authority || canonical_emergency_stop);
}

const char * to_string(const FinalAuthorityClass authority) noexcept
{
  switch (authority) {
    case FinalAuthorityClass::CertifiedNormalSolution:
      return "certified-normal-solution";
    case FinalAuthorityClass::LegacyNormalBypass:
      return "legacy-normal-bypass";
    case FinalAuthorityClass::EmergencyOverride:
      return "emergency-override";
    case FinalAuthorityClass::RecoveryOverride:
      return "recovery-override";
    case FinalAuthorityClass::ControlDisabled:
      return "control-disabled";
  }
  return "unknown";
}

FinalControlDecision resolve_final_control_decision(
  const FinalControlDecisionRequest & request) noexcept
{
  FinalControlDecision decision;
  decision.decision_id = request.decision_id;
  decision.authority = request.authority;
  decision.source = request.source;
  decision.retained_solution = request.retained_solution;
  if (request.problem.has_value() && problem_context_complete(request.problem.value())) {
    decision.intent = request.problem->intent;
    decision.formulation = request.problem->formulation;
    decision.problem_fingerprint = request.problem->fingerprint;
  }
  if (request.solution.has_value()) {
    decision.solution_id = request.solution->solution_id;
  }

  if (request.decision_id == 0U) {
    decision.reason = "missing-decision-id";
    return decision;
  }
  if (request.source.empty()) {
    decision.reason = "missing-control-source";
    return decision;
  }
  if (
    request.authority == FinalAuthorityClass::EmergencyOverride ||
    request.authority == FinalAuthorityClass::RecoveryOverride ||
    request.authority == FinalAuthorityClass::ControlDisabled)
  {
    if (request.supervisor_intent != ControlIntent::Unknown) {
      decision.intent = request.supervisor_intent;
    }
    decision.identity_complete = true;
    decision.canonical_contract_satisfied = true;
    decision.reason = "explicit-supervisor-override";
    return decision;
  }
  if (request.authority == FinalAuthorityClass::LegacyNormalBypass) {
    decision.identity_complete = true;
    decision.reason = "legacy-normal-bypass";
    return decision;
  }
  if (!request.problem.has_value()) {
    decision.reason = "missing-problem-context";
    return decision;
  }
  if (!problem_context_complete(request.problem.value())) {
    decision.reason = "incomplete-problem-context";
    return decision;
  }
  if (
    !request.retained_solution &&
    request.problem->decision_id != request.decision_id)
  {
    decision.reason = "decision-problem-id-mismatch";
    return decision;
  }
  if (!request.solution.has_value()) {
    decision.reason = "missing-certified-solution";
    return decision;
  }
  if (!solution_certified(request.solution.value())) {
    decision.reason = "solution-not-certified";
    return decision;
  }
  if (request.solution->problem_fingerprint != request.problem->fingerprint) {
    decision.reason = "problem-solution-fingerprint-mismatch";
    return decision;
  }
  if (request.solution->formulation != request.problem->formulation) {
    decision.reason = "problem-solution-formulation-mismatch";
    return decision;
  }
  if (canonical_normal_formulation_supported(request.problem->formulation)) {
    if (!request.canonical_normal_command.has_value()) {
      decision.reason = "missing-canonical-command-identity";
      return decision;
    }
    const auto & command = request.canonical_normal_command.value();
    const auto expected_source = request.retained_solution ?
      CanonicalNormalAuthoritySource::RetainedCertified :
      CanonicalNormalAuthoritySource::FreshCertified;
    if (
      command.decision_id != request.decision_id ||
      command.execution_certificate_decision_id != request.decision_id ||
      command.execution_plan_id == 0U ||
      command.problem_fingerprint != request.problem->fingerprint ||
      command.solution_id != request.solution->solution_id ||
      command.formulation != request.problem->formulation ||
      command.intent != request.problem->intent ||
      command.retained_solution != request.retained_solution ||
      command.source != expected_source)
    {
      decision.reason = "canonical-command-identity-mismatch";
      return decision;
    }
    decision.execution_plan_id = command.execution_plan_id;
    decision.execution_certificate_decision_id =
      command.execution_certificate_decision_id;
    decision.canonical_source = command.source;
  }
  decision.identity_complete = true;
  decision.canonical_contract_satisfied =
    canonical_normal_formulation_supported(request.problem->formulation);
  decision.problem_fingerprint = request.problem->fingerprint;
  decision.solution_id = request.solution->solution_id;
  decision.reason = decision.canonical_contract_satisfied ?
    "matching-certified-solution" :
    "matching-certified-noncanonical-formulation";
  return decision;
}

std::string format_final_control_decision(
  const FinalControlDecision & decision)
{
  std::ostringstream stream;
  stream << "MPCC execution contract: decision=" << decision.decision_id
         << ", authority=" << to_string(decision.authority)
         << ", source=" << (decision.source.empty() ? "<none>" : decision.source)
         << ", intent=" << to_string(decision.intent)
         << ", formulation=" << to_string(decision.formulation)
         << ", problem=" << fingerprint_text(decision.problem_fingerprint)
         << ", solution=" << decision.solution_id
         << ", plan=" << decision.execution_plan_id
         << ", execution_certificate_decision="
         << decision.execution_certificate_decision_id
         << ", canonical_source=" << to_string(decision.canonical_source)
         << ", retained=" << (decision.retained_solution ? 1 : 0)
         << ", identity=" << (decision.identity_complete ? "complete" : "incomplete")
         << ", canonical="
         << (decision.canonical_contract_satisfied ? "satisfied" : "violated")
         << ", reason=" << decision.reason;
  return stream.str();
}

}  // namespace multi_purpose_mpc_ros::mpcc_execution_contract
