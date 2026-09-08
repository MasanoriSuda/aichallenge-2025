#include "multi_purpose_mpc_ros/mpcc_architecture_comparison.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <limits>
bool diagnostic_restrict_model_domain=true;
YAML::Node diagnostic_linearization_observations(YAML::NodeType::Sequence);
int main(int argc,char **argv) {
  if(argc!=3)return 2;
  namespace m=multi_purpose_mpc_ros;
  namespace c=m::mpcc_architecture_comparison;
  std::string detail;
  auto record=m::mpcc_architecture_snapshot::load_recorded_interaction_snapshot(argv[1],&detail);
  if(!record)throw std::runtime_error(detail);
  const auto result=c::compare(*record);
  YAML::Node report;
  report["source_fingerprint"]=record->interaction_fingerprint;
  for(const auto &arm:result.arms) {
    YAML::Node row;
    row["arm"]=c::to_string(arm.arm);row["stage"]=c::to_string(arm.stage);
    row["solver"]=m::mpcc_rate_resolved_shadow::to_string(arm.solver_outcome);
    row["bundle"]=arm.bundle.has_value();row["detail"]=arm.detail;
    row["candidate_fingerprint"]=arm.candidate_fingerprint;
    row["authority"]=false;
    report["arms"].push_back(row);
    std::cout<<row<<'\n';
  }
  YAML::Emitter e;e.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  e<<report;std::ofstream(argv[2])<<e.c_str()<<'\n';
}
