// Same-world counterfactual. Candidate selection changes only a tangent point;
// original raw QP constraints, validation and physical proof remain in place.
#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <limits>

bool diagnostic_restrict_model_domain = false;
YAML::Node diagnostic_linearization_observations(YAML::NodeType::Sequence);
int main(int argc, char ** argv) {
  if (argc != 3) return 2;
  namespace m = multi_purpose_mpc_ros;
  std::string detail;
  auto source=m::mpcc_architecture_snapshot::load_recorded_interaction_snapshot(argv[1],&detail);
  if (!source) throw std::runtime_error(detail);
  YAML::Node report;
  report["source_fingerprint"]=source->interaction_fingerprint;
  for (bool restrict_domain : {false,true}) {
    diagnostic_restrict_model_domain=restrict_domain;
    diagnostic_linearization_observations=YAML::Node(YAML::NodeType::Sequence);
    m::mpcc_rate_resolved_shadow::SolverContext solver;
    const auto result=solver.evaluate(source->source);
    auto row=report[restrict_domain ? "coupled_tangent_domain" : "unchanged"];
    row["outcome"]=m::mpcc_rate_resolved_shadow::to_string(result.outcome);
    row["detail"]=result.detail;
    row["observations"]=diagnostic_linearization_observations;
    row["authority"]=false;
    std::cout << restrict_domain << ' ' << row["outcome"] << ' ' << result.detail << '\n';
  }
  YAML::Emitter emitter;emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  emitter << report;std::ofstream(argv[2]) << emitter.c_str() << '\n';
}
