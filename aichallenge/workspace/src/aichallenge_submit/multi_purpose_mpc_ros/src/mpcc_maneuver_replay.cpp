#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_stateless_maneuver.hpp"

#include <filesystem>
#include <iostream>
#include <string>

namespace architecture =
  multi_purpose_mpc_ros::mpcc_architecture_snapshot;
namespace maneuver = multi_purpose_mpc_ros::mpcc_stateless_maneuver;

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: mpcc_maneuver_replay <snapshot.yaml>\n";
    return 2;
  }
  std::string detail;
  const auto recorded = architecture::load_recorded_interaction_snapshot(
    std::filesystem::path{argv[1]}, &detail);
  if (!recorded.has_value()) {
    std::cerr << "load rejected: " << detail << '\n';
    return 3;
  }
  bool any = false;
  for (const int side : {-1, 1}) {
    const auto result = maneuver::build(
      recorded->source, recorded->interaction_fingerprint, side);
    std::cout << "side=" << side
              << " result=" << maneuver::to_string(result.reason);
    if (result.seed.has_value()) {
      any = true;
      std::cout << " candidate_fingerprint="
                << result.seed->candidate_fingerprint
                << " successor="
                << maneuver::to_string(result.seed->terminal_successor)
                << " encounter_stages="
                << result.seed->predicted_encounter_stage_count;
    }
    std::cout << " detail=" << result.detail << '\n';
  }
  return any ? 0 : 4;
}
