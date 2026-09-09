#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_lattice_shadow.hpp"
#include "../20260909-mpcc-complete-rest-candidates/artifact_capture.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <limits>
bool diagnostic_bind_solved_tolerance=false;
YAML::Node diagnostic_certificate;
std::shared_ptr<const multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact::ExecutionArtifact> diagnostic_execution;
int main(int argc,char **argv) {
  if(argc!=3)return 2;
  namespace m=multi_purpose_mpc_ros;
  std::string detail;
  const auto recorded=m::mpcc_architecture_snapshot::load_recorded_interaction_snapshot(argv[1],&detail);
  if(!recorded)throw std::runtime_error(detail);
  YAML::Node report;
  report["source_fingerprint"]=recorded->interaction_fingerprint;
  for(bool bind:{false,true}) {
    diagnostic_bind_solved_tolerance=bind;diagnostic_certificate=YAML::Node();diagnostic_execution.reset();
    m::mpcc_rate_resolved_shadow::SolverContext solver;
    const auto result=m::mpcc_rate_resolved_stop_lattice_shadow::evaluate_current_world(
      recorded->source,solver,{},m::mpcc_rate_resolved_stop_lattice_shadow::EvaluationMode::DirectSevenStateOnly);
    const auto name=bind ? "post_solve_owner" : "unchanged";
    auto row=report[name];row["outcome"]=m::mpcc_rate_resolved_stop_lattice_shadow::to_string(result.reason);
    row["detail"]=result.detail;row["diagnostic"]=diagnostic_certificate;
    row["authority"]=false;
    if(diagnostic_execution)capture_execution(*diagnostic_execution,(std::string{name}+"-artifact.yaml").c_str());
    std::cout<<name<<' '<<row<<'\n';
  }
  YAML::Emitter e;e.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  e<<report;std::ofstream(argv[2])<<e.c_str()<<'\n';
}
