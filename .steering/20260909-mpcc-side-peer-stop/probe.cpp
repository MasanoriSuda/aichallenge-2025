// Standalone diagnostic: no runtime store, publisher, or authority.
#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_control_lattice.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
namespace m = multi_purpose_mpc_ros;
namespace snap = m::mpcc_architecture_snapshot;
namespace osqp = m::persistent_osqp;
YAML::Node vector_node(const Eigen::VectorXd & v) {
  YAML::Node n(YAML::NodeType::Sequence);
  for (int i=0; i<v.size(); ++i) n.push_back(v[i]);
  return n;
}
YAML::Node telemetry(const osqp::SolveTelemetry & t) {
  YAML::Node n;
  n["status"] = t.status; n["iterations"] = t.iterations;
  n["primal_residual"] = t.primal_residual; n["dual_residual"] = t.dual_residual;
  n["scaling_iterations"] = t.scaling_iterations; n["rho_updates"] = t.rho_updates;
  n["rho_estimate"] = t.rho_estimate; n["scaled_termination"] = t.scaled_termination;
  n["absolute_tolerance"] = t.absolute_tolerance; n["relative_tolerance"] = t.relative_tolerance;
  n["total_ms"] = t.total_ms; n["physical_global_tolerance"] = t.physical_global_tolerance;
  return n;
}
void save(const std::filesystem::path & path, const YAML::Node & n) {
  YAML::Emitter e; e.SetDoublePrecision(17); e << n;
  std::ofstream f(path); f << e.c_str() << '\n';
}
int main(int argc, char **argv) {
  if (argc != 4) return 2;
  const std::string mode(argv[1]); const std::filesystem::path out(argv[3]);
  std::filesystem::create_directories(out);
  std::string detail;
  auto record = snap::load_recorded_interaction_snapshot(argv[2], &detail);
  if (!record) throw std::runtime_error(detail);
  YAML::Node report;
  report["source_fingerprint"] = record->interaction_fingerprint;
  report["scope"] = "offline-only; not published and no execution authority";
  report["mode"] = mode;
  if (mode == "qp") {
    if (!record->recorded_qp) throw std::runtime_error("exact QP absent");
    const auto & qp = record->recorded_qp->problem;
    for (const bool equil : {false,true}) for (const bool warm : {false,true}) {
      osqp::PersistentOsqpSolver solver(equil ?
        osqp::ConstraintPreconditioningPolicy::RowToleranceNormalizedWithInternalEquilibration :
        osqp::ConstraintPreconditioningPolicy::RowToleranceNormalized);
      auto result = solver.solve(qp.quadratic_cost, qp.constraints, qp.linear_cost,
        qp.lower_bound, qp.upper_bound, warm ? record->recorded_qp->warm_start : std::nullopt,
        qp.variable_scaling);
      const std::string name = std::string(equil ? "equilibrated" : "normalized")+(warm ? "-warm" : "-cold");
      auto n = telemetry(result.telemetry);
      n["solved"] = result.result.has_value(); n["detail"] = result.failure_detail;
      n["warm_start_available"] = record->recorded_qp->warm_start.has_value();
      if (result.result) {
        n["primal"] = vector_node(result.result->primal);
        n["dual"] = vector_node(result.result->dual);
        n["maximum_normalized_violation"] = result.result->maximum_normalized_constraint_violation;
      } else if (result.rejected_primal) n["rejected_primal"] = vector_node(*result.rejected_primal);
      report["arms"][name] = n;
    }
  } else {
    m::mpcc_rate_resolved_shadow::SolverContext solver;
    auto stop = m::mpcc_rate_resolved_stop_control_lattice::build_current_world_maximum_braking_candidate(
      record->source, solver.physical_constraint_tolerance());
    if (!stop.accepted()) throw std::runtime_error(stop.detail);
    if (mode == "historical") {
      for (std::size_t i=0; i<stop.candidate.request.states.size(); ++i) {
        stop.candidate.request.states[i].weight = record->source.request.states[i].weight;
        stop.candidate.request.states[i].linear_cost = record->source.request.states[i].linear_cost;
      }
      for (std::size_t i=0; i<stop.candidate.request.inputs.size(); ++i) {
        stop.candidate.request.inputs[i].weight = record->source.request.inputs[i].weight;
        stop.candidate.request.inputs[i].linear_cost = record->source.request.inputs[i].linear_cost;
      }
      stop.candidate.request.input_delta_weight = record->source.request.input_delta_weight;
    } else if (mode != "zero") throw std::runtime_error("unknown mode");
    const auto saved = snap::record_proof_failure(stop.candidate,
      snap::PipelineStage::PhysicalProof, "offline-stop-candidate-input",
      "comparison input only; neither published nor a runtime failure", out/"input");
    report["candidate_snapshot"] = saved.snapshot_file.string();
    report["candidate_fingerprint"] = snap::fingerprint_interaction_snapshot(stop.candidate);
    const auto result = mode == "historical" ? solver.evaluate_stop_physical_support_audit(stop.candidate) : solver.evaluate(stop.candidate);
    report["solved"] = result.solved; report["detail"] = result.detail;
    report["telemetry"] = telemetry(result.solver);
    if (result.latest_state_feedback_preparation)
      report["prepared_primal"] = vector_node(result.latest_state_feedback_preparation->prepared_primal);
    if (result.execution_artifact) {
      const auto & a = *result.execution_artifact;
      report["artifact"]["physical_global_tolerance"] = a.physical_global_tolerance;
      const auto state = [](const auto & s) {
        YAML::Node n;
        for (double v : {s.lateral_m,s.lag_m,s.heading_offset_rad,s.velocity_mps,s.progress_m,s.steering_rad,s.response_steering_rad}) n.push_back(v);
        return n;
      };
      report["artifact"]["semantic_initial_state"] = state(*a.semantic_initial_state);
      for (const auto & s : a.predicted_states) report["artifact"]["raw_states"].push_back(state(s));
      for (const auto & c : a.control_stages) {
        YAML::Node n;
        for (double v : {c.acceleration_mps2,c.steering_rate_radps,c.virtual_progress_speed_mps,c.duration_sec,
          c.virtual_progress_lower_mps,c.virtual_progress_upper_mps,c.acceleration_lower_mps2,c.acceleration_upper_mps2,c.path_curvature_radpm}) n.push_back(v);
        report["artifact"]["control_stages"].push_back(n);
      }
    }
  }
  save(out/"report.yaml",report);
  std::cout << "saved " << (out/"report.yaml") << std::endl;
}
