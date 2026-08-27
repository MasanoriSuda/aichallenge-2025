#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"

#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char ** argv)
{
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: mpcc_qp_replay SNAPSHOT_YAML [--cold|--warm]\n";
    return 2;
  }
  bool warm = true;
  if (argc == 3) {
    const std::string mode{argv[2]};
    if (mode == "--cold") {
      warm = false;
    } else if (mode != "--warm") {
      std::cerr << "unknown replay mode: " << mode << '\n';
      return 2;
    }
  }
  const auto replay =
    multi_purpose_mpc_ros::mpcc_architecture_snapshot::replay_recorded_qp(
    std::filesystem::path{argv[1]}, warm);
  std::cout << "loaded=" << replay.loaded
            << " mode=" << (warm ? "warm" : "cold")
            << " warm_available=" << replay.warm_start_available
            << " solved=" << replay.outcome.result.has_value()
            << " status=" << replay.outcome.telemetry.status
            << " iterations=" << replay.outcome.telemetry.iterations
            << " total_ms=" << replay.outcome.telemetry.total_ms
            << " detail=" << replay.detail << '\n';
  return replay.loaded ? 0 : 2;
}
