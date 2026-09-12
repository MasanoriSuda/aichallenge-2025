#pragma once

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

#include <array>
#include <optional>

namespace multi_purpose_mpc_ros::mpcc_native_initialization {

namespace shadow = mpcc_rate_resolved_shadow;
namespace adapter = mpcc_rate_resolved_adapter;

/// The target-dependent populations retain their own candidate ownership.
/// This bounded population is only for an ordinary target-free Cruise source.
inline bool eligible(const shadow::Snapshot & source) noexcept
{
  const auto & context = source.identity.source_context;
  return context.formulation == mpcc_execution_contract::Formulation::VelocitySteeringTireBodyProgress9State &&
    context.intent == mpcc_execution_contract::ControlIntent::Cruise &&
    context.target_id.empty() && context.execution_side_sign == 0 &&
    !context.dynamic_obstacle_constraint_active &&
    !source.dynamic_obstacle_refinement_active &&
    !source.request.maximum_braking_feasibility &&
    source.request.initial_tangent_policy == adapter::InitialTangentPolicy::CurrentSteering;
}

struct Population
{
  /// Fixed order: reference-steering/rest launch, then original initialization.
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
    adapter::InitialTangentPolicy::ReferenceSteeringWithRestLaunch;
  population.candidates[1].identity.sequence = second_sequence;
  return population;
}

}  // namespace multi_purpose_mpc_ros::mpcc_native_initialization
