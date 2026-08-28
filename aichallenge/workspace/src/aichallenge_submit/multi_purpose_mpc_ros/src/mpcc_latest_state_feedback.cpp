#include "multi_purpose_mpc_ros/mpcc_latest_state_feedback.hpp"

#include <algorithm>
#include <cmath>

namespace multi_purpose_mpc_ros::mpcc_latest_state_feedback
{

const char * to_string(const Reason reason) noexcept
{
  switch (reason) {
    case Reason::InvalidInput: return "invalid-input";
    case Reason::EmptyEnvelope: return "empty-envelope";
    case Reason::PreparedCommandReachable: return "prepared-reachable";
    case Reason::ProjectedToReachableEnvelope: return "projected";
  }
  return "unknown";
}

Result solve(const Request & request) noexcept
{
  Result result;
  result.prepared_steering_rad = request.prepared_steering_rad;
  if (
    !std::isfinite(request.previous_published_steering_rad) ||
    !std::isfinite(request.prepared_steering_rad) ||
    !std::isfinite(request.maximum_abs_steering_rad) ||
    request.maximum_abs_steering_rad <= 0.0 ||
    !std::isfinite(request.maximum_abs_steering_rate_radps) ||
    request.maximum_abs_steering_rate_radps < 0.0 ||
    !std::isfinite(request.publication_age_sec) ||
    request.publication_age_sec < 0.0 ||
    !std::isfinite(request.physical_certificate_tolerance) ||
    request.physical_certificate_tolerance < 0.0)
  {
    return result;
  }

  const double maximum_step_rad =
    request.maximum_abs_steering_rate_radps * request.publication_age_sec +
    request.physical_certificate_tolerance;
  result.lower_rad = std::max(
    -request.maximum_abs_steering_rad,
    request.previous_published_steering_rad - maximum_step_rad);
  result.upper_rad = std::min(
    request.maximum_abs_steering_rad,
    request.previous_published_steering_rad + maximum_step_rad);
  if (
    !std::isfinite(result.lower_rad) || !std::isfinite(result.upper_rad) ||
    result.lower_rad > result.upper_rad)
  {
    result.reason = Reason::EmptyEnvelope;
    return result;
  }

  result.feedback_steering_rad = std::clamp(
    request.prepared_steering_rad, result.lower_rad, result.upper_rad);
  result.correction_rad =
    result.feedback_steering_rad - request.prepared_steering_rad;
  result.reason = result.correction_rad == 0.0 ?
    Reason::PreparedCommandReachable :
    Reason::ProjectedToReachableEnvelope;
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_latest_state_feedback
