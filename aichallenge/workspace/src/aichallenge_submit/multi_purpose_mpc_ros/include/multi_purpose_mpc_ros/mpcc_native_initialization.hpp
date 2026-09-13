#pragma once

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace multi_purpose_mpc_ros::mpcc_native_initialization {

namespace shadow = mpcc_rate_resolved_shadow;
namespace adapter = mpcc_rate_resolved_adapter;

/// The target-dependent populations retain their own candidate ownership.
/// This bounded population owns target-free Cruise and forward Rejoin sources.
inline bool eligible(const shadow::Snapshot & source) noexcept
{
  const auto & context = source.identity.source_context;
  const bool forward_rejoin = context.intent == mpcc_execution_contract::ControlIntent::Rejoin &&
    source.request.states.size() > 1U &&
    std::any_of(source.request.states.begin() + 1, source.request.states.end(), [](const auto &stage) {
      const double reference = stage.reference[mpcc_rate_resolved::kVelocityIndex];
      const double lower = stage.lower[mpcc_rate_resolved::kVelocityIndex];
      const double upper = stage.upper[mpcc_rate_resolved::kVelocityIndex];
      return std::isfinite(reference) && !std::isnan(lower) && !std::isnan(upper) &&
        lower <= upper && std::clamp(reference, lower, upper) > 0.0;
    });
  return context.formulation == mpcc_execution_contract::Formulation::VelocitySteeringTireBodyProgress9State &&
    (context.intent == mpcc_execution_contract::ControlIntent::Cruise || forward_rejoin) &&
    context.target_id.empty() && context.execution_side_sign == 0 &&
    !context.dynamic_obstacle_constraint_active &&
    !source.dynamic_obstacle_refinement_active &&
    !source.request.maximum_braking_feasibility &&
    source.request.initial_tangent_policy == adapter::InitialTangentPolicy::CurrentSteering;
}

struct Population
{
  /// Fixed order: intent-specific numerical tangent, then original initialization.
  /// Both members exist before either solve and have distinct reserved IDs.
  std::array<shadow::Snapshot, 2> candidates;
};

inline std::optional<Population> build(
  const shadow::Snapshot & source, std::uint64_t second_sequence)
{
  if (!eligible(source) || !shadow::artifact::identity_valid(source.identity) ||
      second_sequence <= source.identity.sequence ||
      second_sequence - source.identity.sequence != 1U) return std::nullopt;
  Population population{{source, source}};
  population.candidates[0].request.initial_tangent_policy =
    source.identity.source_context.intent == mpcc_execution_contract::ControlIntent::Rejoin ?
    adapter::InitialTangentPolicy::SteeringBeforeDrive :
    adapter::InitialTangentPolicy::ReferenceSteeringWithRestLaunch;
  population.candidates[1].identity.sequence = second_sequence;
  return population;
}

}  // namespace multi_purpose_mpc_ros::mpcc_native_initialization
