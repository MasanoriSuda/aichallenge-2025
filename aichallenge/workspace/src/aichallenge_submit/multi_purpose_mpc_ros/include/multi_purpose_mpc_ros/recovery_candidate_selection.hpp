#pragma once

#include "multi_purpose_mpc_ros/recovery_footprint.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace multi_purpose_mpc_ros::recovery_footprint
{

/// Candidate preference changes evaluation order, never physical acceptance.
template<class Preferred, class Remaining>
std::optional<FeasibilityResult> select_preferred_then_remaining(
  const bool prefer, Preferred && preferred, Remaining && remaining)
{
  if (prefer) {
    if (auto selected = preferred()) return selected;
  }
  return remaining();
}

struct HeadingAlignedReverseRequest
{
  double initial_yaw_rad{};
  double heading_error_rad{};
  double maximum_steering_angle_rad{};
  std::size_t steering_sample_count{};
  bool current_footprint_clear{false};
  bool candidate_committed{false};
  std::optional<double> committed_steering_angle_rad;
  std::optional<double> desired_steering_angle_rad;
};

struct HeadingAlignedReverseSelection
{
  std::optional<FeasibilityResult> first;
  std::optional<FeasibilityResult> selected;
};

/// Select a constant-steering Recovery primitive using the caller's unchanged
/// physical evaluator and course predicate. This supplies no normal authority.
template<class Evaluate, class CourseAllowed>
HeadingAlignedReverseSelection select_heading_aligned_reverse(
  const HeadingAlignedReverseRequest & request,
  Evaluate && evaluate, CourseAllowed && course_allowed)
{
  HeadingAlignedReverseSelection selection;
  double best_score = std::numeric_limits<double>::infinity();
  const auto wrap = [](const double angle) {
      return std::atan2(std::sin(angle), std::cos(angle));
    };
  const auto consider = [&](const ReversePrimitive primitive, const double magnitude) {
      auto result = evaluate(primitive, magnitude);
      if (!selection.first) selection.first = result;
      if (!result.feasible || result.rollout.empty() || !course_allowed(result)) return;
      const double yaw_delta = wrap(result.rollout.back().pose.yaw_rad - request.initial_yaw_rad);
      const double heading_error = std::isfinite(request.heading_error_rad) ?
        std::abs(wrap(request.heading_error_rad + yaw_delta)) : std::abs(yaw_delta);
      const double score = request.desired_steering_angle_rad ?
        std::abs(result.steering_angle_rad - *request.desired_steering_angle_rad) +
        0.10 * heading_error : heading_error;
      if (!selection.selected || score + 1e-12 < best_score) {
        best_score = score;
        selection.selected = std::move(result);
      }
    };
  // Guidance affects ranking, not the available clear-footprint primitives.
  // Committed maneuvers and contact-only selection retain their existing scope.
  if (!request.candidate_committed &&
    (request.current_footprint_clear || request.desired_steering_angle_rad))
  {
    consider(ReversePrimitive::Straight, 0.0);
    const auto samples = steering_magnitude_samples(
      request.maximum_steering_angle_rad, request.steering_sample_count);
    for (const auto primitive : {ReversePrimitive::Left, ReversePrimitive::Right}) {
      for (const double magnitude : samples) consider(primitive, magnitude);
    }
  } else {
    const double magnitude = request.committed_steering_angle_rad ?
      std::abs(*request.committed_steering_angle_rad) : request.maximum_steering_angle_rad;
    for (const auto primitive : {
        ReversePrimitive::Straight, ReversePrimitive::Left, ReversePrimitive::Right})
    {
      consider(primitive, magnitude);
    }
  }
  return selection;
}

}  // namespace multi_purpose_mpc_ros::recovery_footprint
