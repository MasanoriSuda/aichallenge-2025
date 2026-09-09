"""Deterministic source-copy hypothesis; does not modify production."""
def transform(source):
    helper = r'''
// A tangent owns a transition, so its state and virtual speed must jointly
// lie in the immutable course domain. Soft references and QP residuals can
// both violate this domain even when each independent box is respected.
// Select only the tangent input; never rewrite costs, bounds or a solution.
std::optional<double> select_virtual_speed_tangent(
  const mpcc_rate_resolved::CourseFrame & frame,
  const double progress, const double dt,
  const double reference, const double lower, const double upper) noexcept
{
  if (!frame.knots) {
    return reference;
  }
  if (frame.knots->size() < 2U || !std::isfinite(dt) || dt <= 0.0 ||
    !std::isfinite(progress) || !std::isfinite(reference))
  {
    return std::nullopt;
  }
  const double minimum = std::max(lower,
    (frame.knots->front().progress_m - frame.progress_origin_m - progress) / dt);
  const double maximum = std::min(upper,
    (frame.knots->back().progress_m - frame.progress_origin_m - progress) / dt);
  if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum > maximum) {
    return std::nullopt;
  }
  return std::clamp(reference, minimum, maximum);
}

'''
    anchor='double steering_from_curvature('
    assert source.count(anchor)==1
    source=source.replace(anchor,helper+anchor)
    anchor='    const auto linearization = model::linearize_temporal_frenet(\n      model::LinearizationRequest{\n        state_reference[0]'
    assert source.count(anchor)==1
    probe='''    const auto virtual_speed_tangent = select_virtual_speed_tangent(
      request.course_frame, state_reference[model::kProgressIndex],
      legacy_input.stage_dt_sec, legacy_input.reference[2],
      legacy_input.lower[2], legacy_input.upper[2]);
    if (!virtual_speed_tangent) {
      return reject(RejectReason::LinearizationUnavailable, stage);
    }
'''
    source=source.replace(anchor,probe+anchor)
    anchor='legacy_input.reference[0], 0.0, legacy_input.reference[2],'
    assert source.count(anchor)==1
    source=source.replace(anchor,'legacy_input.reference[0], 0.0, *virtual_speed_tangent,')
    start=source.index('    if (request.course_frame.knots) {',source.index('RelinearizationResult relinearize_around_primal('))
    end=source.index('    const auto linearization = model::linearize_temporal_frenet(',start)
    source=source[:start]+'''    const auto virtual_speed_tangent = select_virtual_speed_tangent(
      request.course_frame, linearization_state[model::kProgressIndex],
      semantic_input.stage_dt_sec, linearization_input[model::kVirtualProgressSpeedIndex],
      problem.input_lower[problem_input + model::kVirtualProgressSpeedIndex],
      problem.input_upper[problem_input + model::kVirtualProgressSpeedIndex]);
    if (!virtual_speed_tangent) {
      result.reason = RelinearizationReason::LinearizationUnavailable;
      result.stage = stage;
      return result;
    }
    linearization_input[model::kVirtualProgressSpeedIndex] = *virtual_speed_tangent;
'''+source[end:]
    return source
