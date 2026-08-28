#include "multi_purpose_mpc_ros/mpcc_architecture_comparison.hpp"

#include <Eigen/Dense>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace architecture =
  multi_purpose_mpc_ros::mpcc_architecture_snapshot;
namespace comparison =
  multi_purpose_mpc_ros::mpcc_architecture_comparison;

int main(int argc, char ** argv)
{
  const bool wall_restoration_only =
    argc == 3 && std::string{argv[2]} == "--wall-restoration-only";
  const bool wall_buckets_only =
    argc == 3 && std::string{argv[2]} == "--wall-buckets-only";
  const bool external_primal =
    argc == 4 && std::string{argv[2]} == "--external-primal";
  if (
    argc != 2 && !wall_restoration_only && !wall_buckets_only &&
    !external_primal)
  {
    std::cerr << "usage: mpcc_architecture_compare <snapshot.yaml> "
                 "[--wall-restoration-only | --wall-buckets-only | "
                 "--external-primal <values.txt>]\n";
    return 2;
  }
  std::string detail;
  const auto recorded = architecture::load_recorded_interaction_snapshot(
    std::filesystem::path{argv[1]}, &detail);
  if (!recorded.has_value()) {
    std::cerr << "load rejected: " << detail << '\n';
    return 3;
  }
  comparison::Report report;
  if (external_primal) {
    std::ifstream input{argv[3]};
    std::vector<double> values;
    double value = 0.0;
    while (input >> value) {
      values.push_back(value);
    }
    if (!input.eof() || values.empty()) {
      std::cerr << "external primal rejected: unreadable or empty file\n";
      return 5;
    }
    Eigen::VectorXd primal(static_cast<Eigen::Index>(values.size()));
    for (std::size_t index = 0U; index < values.size(); ++index) {
      primal[static_cast<Eigen::Index>(index)] = values[index];
    }
    report = comparison::verify_external_primal(recorded.value(), primal);
  } else if (wall_restoration_only) {
    report = comparison::compare_wall_restoration(recorded.value());
  } else if (wall_buckets_only) {
    report = comparison::compare_wall_buckets(recorded.value());
  } else {
    report = comparison::compare(recorded.value());
  }
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
              << " external_violation=" <<
      arm.maximum_external_constraint_violation
              << " external_normalized=" <<
      arm.maximum_external_normalized_constraint_violation
              << " external_row=" <<
      arm.maximum_external_normalized_constraint_row
              << " lattice_transition=" << arm.lattice_transition_stage
              << " lattice_ahead=" << arm.lattice_ahead_stage
              << " direct_final=" << arm.direct_final_attempted
              << " direct_stage=" << comparison::to_string(
        arm.direct_final_stage)
              << " continuation=" << arm.continuation_attempted
              << " continuation_steps=" <<
      arm.continuation_solved_step_count
              << " continuation_ms=" << arm.continuation_compute_ms
              << " candidate_source=" << arm.candidate_source
              << " candidate_count=" << arm.candidate_count
              << " bundle=" << arm.bundle.has_value()
              << " detail=" << arm.detail << '\n';
  }
  return any_bundle ? 0 : 4;
}
