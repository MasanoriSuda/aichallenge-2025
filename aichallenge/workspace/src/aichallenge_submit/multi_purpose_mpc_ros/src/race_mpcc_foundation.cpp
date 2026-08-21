#include <multi_purpose_mpc_ros/race_mpcc_foundation.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace multi_purpose_mpc_ros::race_mpcc_foundation
{

const char * homotopy_name(const Homotopy homotopy) noexcept
{
  switch (homotopy) {
    case Homotopy::Left:
      return "left";
    case Homotopy::Right:
      return "right";
    case Homotopy::Hold:
      return "hold";
    case Homotopy::Return:
      return "return";
    case Homotopy::None:
      return "none";
  }
  return "unknown";
}

const char * target_provenance_stage_name(
  const TargetProvenanceStage stage) noexcept
{
  switch (stage) {
    case TargetProvenanceStage::None:
      return "none";
    case TargetProvenanceStage::Observed:
      return "observed";
    case TargetProvenanceStage::Locked:
      return "locked";
  }
  return "unknown";
}

const char * target_provenance_reject_reason_name(
  const TargetProvenanceRejectReason reason) noexcept
{
  switch (reason) {
    case TargetProvenanceRejectReason::None:
      return "none";
    case TargetProvenanceRejectReason::InvalidExpected:
      return "invalid-expected";
    case TargetProvenanceRejectReason::InvalidCurrent:
      return "invalid-current";
    case TargetProvenanceRejectReason::TargetMismatch:
      return "target-mismatch";
    case TargetProvenanceRejectReason::SourceRegression:
      return "source-regression";
    case TargetProvenanceRejectReason::ReceiptRegression:
      return "receipt-regression";
    case TargetProvenanceRejectReason::GenerationRegression:
      return "generation-regression";
    case TargetProvenanceRejectReason::StageRegression:
      return "stage-regression";
    case TargetProvenanceRejectReason::ProgressDelta:
      return "progress-delta";
    case TargetProvenanceRejectReason::LateralDelta:
      return "lateral-delta";
  }
  return "unknown";
}

namespace
{

bool finite_provenance(const TargetProvenance & provenance) noexcept
{
  return provenance.valid && !provenance.target_id.empty() &&
         std::isfinite(provenance.source_stamp_sec) &&
         std::isfinite(provenance.receipt_sec) &&
         std::isfinite(provenance.course_progress_m) &&
         std::isfinite(provenance.course_lateral_m) &&
         provenance.observation_generation > 0U &&
         provenance.stage != TargetProvenanceStage::None;
}

double circular_delta(
  const double current, const double expected,
  const bool circular, const double path_length_m) noexcept
{
  double delta = current - expected;
  if (!circular || !std::isfinite(path_length_m) || path_length_m <= 0.0) {
    return delta;
  }
  while (delta > 0.5 * path_length_m) {
    delta -= path_length_m;
  }
  while (delta < -0.5 * path_length_m) {
    delta += path_length_m;
  }
  return delta;
}

}  // namespace

TargetProvenanceValidation validate_target_provenance(
  const TargetProvenanceValidationRequest & request) noexcept
{
  TargetProvenanceValidation result;
  if (!finite_provenance(request.expected)) {
    result.reject_reason = TargetProvenanceRejectReason::InvalidExpected;
    return result;
  }
  if (!finite_provenance(request.current)) {
    result.reject_reason = TargetProvenanceRejectReason::InvalidCurrent;
    return result;
  }
  if (request.expected.target_id != request.current.target_id) {
    result.reject_reason = TargetProvenanceRejectReason::TargetMismatch;
    return result;
  }
  if (request.current.source_stamp_sec + 1e-9 < request.expected.source_stamp_sec) {
    result.reject_reason = TargetProvenanceRejectReason::SourceRegression;
    return result;
  }
  if (request.current.receipt_sec + 1e-9 < request.expected.receipt_sec) {
    result.reject_reason = TargetProvenanceRejectReason::ReceiptRegression;
    return result;
  }
  if (request.current.observation_generation < request.expected.observation_generation) {
    result.reject_reason = TargetProvenanceRejectReason::GenerationRegression;
    return result;
  }
  if (
    request.expected.stage == TargetProvenanceStage::Locked &&
    request.current.stage != TargetProvenanceStage::Locked)
  {
    result.reject_reason = TargetProvenanceRejectReason::StageRegression;
    return result;
  }
  result.same_observation =
    request.current.observation_generation == request.expected.observation_generation;
  result.progress_delta_m = circular_delta(
    request.current.course_progress_m, request.expected.course_progress_m,
    request.circular, request.path_length_m);
  result.lateral_delta_m =
    request.current.course_lateral_m - request.expected.course_lateral_m;
  if (
    !std::isfinite(result.progress_delta_m) ||
    result.progress_delta_m < -std::max(0.0, request.maximum_backward_progress_m) - 1e-9 ||
    result.progress_delta_m > std::max(0.0, request.maximum_forward_progress_m) + 1e-9)
  {
    result.reject_reason = TargetProvenanceRejectReason::ProgressDelta;
    return result;
  }
  if (
    !std::isfinite(result.lateral_delta_m) ||
    std::abs(result.lateral_delta_m) >
    std::max(0.0, request.maximum_lateral_change_m) + 1e-9)
  {
    result.reject_reason = TargetProvenanceRejectReason::LateralDelta;
    return result;
  }
  result.valid = true;
  result.reject_reason = TargetProvenanceRejectReason::None;
  return result;
}

std::string format_shadow_decision(const ShadowDecision & decision)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2)
         << "epoch=" << decision.context_epoch
         << ",target=" << (decision.target_id.empty() ? "none" : decision.target_id)
         << ",target_provenance=" << (decision.target_provenance.valid ? 1 : 0)
         << '/' << target_provenance_stage_name(decision.target_provenance.stage)
         << '/' << decision.target_provenance.observation_generation
         << '/' << decision.target_provenance.source_stamp_sec
         << '/' << decision.target_provenance.course_progress_m
         << '/' << decision.target_provenance.course_lateral_m
         << ",geometry=" << (decision.stage_geometry_valid ? "valid" : "invalid")
         << '/' << decision.stage_count << '/' << decision.horizon_distance_m << "m";
  for (const auto & candidate : decision.candidates) {
    stream << ',' << homotopy_name(candidate.homotopy) << '='
           << (candidate.attempted ? 1 : 0) << '/'
           << (candidate.feasible ? 1 : 0) << "/warm="
           << (candidate.warm_start_applied ? 1 : 0) << "/reset="
           << (candidate.solver_context_reset ? 1 : 0) << "/count="
           << candidate.solver_context_solve_count << "/obj="
           << candidate.objective << "/p=" << candidate.terminal_progress_m
           << "/v=" << candidate.terminal_velocity_mps << "/wall="
           << candidate.minimum_wall_reserve_m << "/solve=" << candidate.solve_ms
           << "ms/iter=" << candidate.iterations << "/reason=" << candidate.reason;
  }
  stream << ",selected=" << homotopy_name(decision.selected)
         << ",selection_reason=" << decision.selection_reason
         << ",authority=shadow";
  return stream.str();
}

}  // namespace multi_purpose_mpc_ros::race_mpcc_foundation
