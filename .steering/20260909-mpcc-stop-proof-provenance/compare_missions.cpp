#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_lattice_shadow.hpp"
#include "../20260909-mpcc-complete-rest-candidates/artifact_capture.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <limits>
int diagnostic_stop_mission=0;
int main(int argc,char **argv) {
  if(argc!=3)return 2;
  namespace m=multi_purpose_mpc_ros;
  std::string detail;
  const auto record=m::mpcc_architecture_snapshot::load_recorded_interaction_snapshot(argv[1],&detail);
  if(!record)throw std::runtime_error(detail);
  YAML::Node report;
  report["source_fingerprint"]=record->interaction_fingerprint;
  for(int mode:{0,1,2,3}) {
    diagnostic_stop_mission=mode;
    m::mpcc_rate_resolved_shadow::SolverContext solver;
    const auto result=m::mpcc_rate_resolved_stop_lattice_shadow::evaluate_current_world(
      record->source,solver,{},m::mpcc_rate_resolved_stop_lattice_shadow::EvaluationMode::DirectSevenStateOnly);
    YAML::Node row;
    row["mode"]=mode;
    row["controls"]=mode>=2 ? "maximum-braking-law" : "free-through-rest";
    row["objective"]=mode%2 ? "zero-feasibility" : "inherited-quadratic";
    row["outcome"]=m::mpcc_rate_resolved_stop_lattice_shadow::to_string(result.reason);
    row["detail"]=result.detail;row["wall"]=m::mpcc_rate_resolved_physical_wall::to_string(result.wall_outcome);
    row["dynamic_valid"]=result.dynamic_valid;row["dynamic_clear"]=result.dynamic_clear;
    row["minimum_dynamic_clearance_m"]=result.minimum_dynamic_clearance_m;
    row["compute_ms"]=result.total_compute_ms;row["authority"]=false;
    if(result.certified_stop_plan) {
      const auto &plan=*result.certified_stop_plan;
      capture_execution(*plan.execution_artifact,(std::to_string(mode)+"-artifact.yaml").c_str());
      row["candidate_fingerprint"]=m::mpcc_architecture_snapshot::fingerprint_interaction_snapshot(*plan.solver_source_snapshot);
      row["candidate_context_fingerprint"]=plan.execution_artifact->identity.source_context.fingerprint;
      row["terminal_speed_mps"]=plan.execution_artifact->predicted_states.back().velocity_mps;
      row["first_acceleration_mps2"]=plan.execution_artifact->control_stages.front().acceleration_mps2;
      const auto capture=m::mpcc_architecture_snapshot::record_proof_failure(*plan.solver_source_snapshot,
        m::mpcc_architecture_snapshot::PipelineStage::PhysicalProof,"offline-mission-observation",
        "Accepted source capture only; no current-state join or publisher in this diagnostic.","source-"+std::to_string(mode));
      row["source_capture"]=capture.snapshot_file.string();
    }
    report["arms"].push_back(row);std::cout<<row<<'\n';
  }
  YAML::Emitter e;e.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  e<<report;std::ofstream(argv[2])<<e.c_str()<<'\n';
}
