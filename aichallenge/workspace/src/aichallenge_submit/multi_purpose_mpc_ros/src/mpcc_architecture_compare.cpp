#include "multi_purpose_mpc_ros/mpcc_architecture_comparison.hpp"

#include <filesystem>
#include <iostream>
#include <string>

namespace architecture =
  multi_purpose_mpc_ros::mpcc_architecture_snapshot;
namespace comparison =
  multi_purpose_mpc_ros::mpcc_architecture_comparison;

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: mpcc_architecture_compare <snapshot.yaml>\n";
    return 2;
  }
  std::string detail;
  const auto recorded = architecture::load_recorded_interaction_snapshot(
    std::filesystem::path{argv[1]}, &detail);
  if (!recorded.has_value()) {
    std::cerr << "load rejected: " << detail << '\n';
    return 3;
  }
  const auto report = comparison::compare(recorded.value());
  std::cout << "source=" << report.source_interaction_fingerprint
            << " accepted=" << report.source_accepted
            << " detail=" << report.detail << '\n';
  bool any_bundle = false;
  for (const auto & arm : report.arms) {
    any_bundle = any_bundle || arm.bundle.has_value();
    std::cout << "arm=" << comparison::to_string(arm.arm)
              << " stage=" << comparison::to_string(arm.stage)
              << " candidate=" << arm.candidate_fingerprint
              << " solver=" <<
      multi_purpose_mpc_ros::mpcc_rate_resolved_shadow::to_string(
        arm.solver_outcome)
              << " solve_ms=" << arm.solver_compute_ms
              << " terminal_progress=" << arm.terminal_progress_m
              << " terminal_velocity=" << arm.terminal_velocity_mps
              << " lateral_reserve=" << arm.minimum_lateral_bound_reserve_m
              << " lattice_transition=" << arm.lattice_transition_stage
              << " lattice_ahead=" << arm.lattice_ahead_stage
              << " bundle=" << arm.bundle.has_value()
              << " detail=" << arm.detail << '\n';
  }
  return any_bundle ? 0 : 4;
}
