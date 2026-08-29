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
  const bool physical_dynamic_sqp_only =
    argc == 3 && std::string{argv[2]} == "--physical-dynamic-sqp-only";
  const bool proof_guided_dynamic_sqp_only =
    argc == 3 && std::string{argv[2]} ==
    "--proof-guided-dynamic-sqp-only";
  const bool external_primal =
    argc == 4 && std::string{argv[2]} == "--external-primal";
  const bool external_primal_omit_heading =
    argc == 4 && std::string{argv[2]} ==
    "--external-primal-omit-wall-heading-bucket";
  const bool external_primal_omit_lag =
    argc == 4 && std::string{argv[2]} ==
    "--external-primal-omit-wall-lag-bucket";
  const bool external_primal_physical_only =
    argc == 4 && std::string{argv[2]} ==
    "--external-primal-physical-nonlinear-oracle";
  const bool rejected_primal_only =
    argc == 3 && std::string{argv[2]} == "--rejected-primal-only";
  if (
    argc != 2 && !wall_restoration_only && !wall_buckets_only &&
    !physical_dynamic_sqp_only && !proof_guided_dynamic_sqp_only &&
    !external_primal && !external_primal_omit_heading &&
    !external_primal_omit_lag && !external_primal_physical_only &&
    !rejected_primal_only)
  {
    std::cerr << "usage: mpcc_architecture_compare <snapshot.yaml> "
                 "[--wall-restoration-only | --wall-buckets-only | "
                 "--physical-dynamic-sqp-only | "
                 "--proof-guided-dynamic-sqp-only | "
                 "--external-primal <values.txt> | "
                 "--external-primal-omit-wall-heading-bucket <values.txt> | "
                 "--external-primal-omit-wall-lag-bucket <values.txt> | "
                 "--external-primal-physical-nonlinear-oracle "
                 "<values.txt> | --rejected-primal-only]\n";
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
  if (rejected_primal_only) {
    auto primal = recorded->recorded_qp.rejected_primal;
    if (!primal.has_value()) {
      // Older snapshots did not serialize the rejected iterate.  Replaying
      // the immutable exact problem from a cold state is observation-only and
      // reproduces the evidence without changing production authority.
      const auto replay = architecture::replay_recorded_qp(argv[1], false);
      primal = replay.outcome.rejected_primal;
    }
    if (!primal.has_value()) {
      std::cerr << "rejected primal unavailable after exact-QP replay\n";
      return 6;
    }
    report = comparison::verify_external_primal(
      recorded.value(), primal.value(),
      comparison::ExternalPrimalConstraintPolicy::ExactRecorded);
  } else if (
    external_primal || external_primal_omit_heading ||
    external_primal_omit_lag || external_primal_physical_only)
  {
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
    auto policy = comparison::ExternalPrimalConstraintPolicy::ExactRecorded;
    if (external_primal_omit_heading) {
      policy = comparison::ExternalPrimalConstraintPolicy::
        OmitWallHeadingBucket;
    } else if (external_primal_omit_lag) {
      policy = comparison::ExternalPrimalConstraintPolicy::OmitWallLagBucket;
    } else if (external_primal_physical_only) {
      policy = comparison::ExternalPrimalConstraintPolicy::PhysicalNonlinearOracle;
    }
    report = comparison::verify_external_primal(
      recorded.value(), primal, policy);
  } else if (wall_restoration_only) {
    report = comparison::compare_wall_restoration(recorded.value());
  } else if (wall_buckets_only) {
    report = comparison::compare_wall_buckets(recorded.value());
  } else if (physical_dynamic_sqp_only) {
    report = comparison::compare_physical_dynamic_sqp(recorded.value());
  } else if (proof_guided_dynamic_sqp_only) {
    report = comparison::compare_proof_guided_dynamic_sqp(recorded.value());
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
              << " sqp_depth=" << arm.dynamic_sqp_depth
              << " bundle=" << arm.bundle.has_value()
              << " detail=" << arm.detail << '\n';
  }
  return any_bundle ? 0 : 4;
}
