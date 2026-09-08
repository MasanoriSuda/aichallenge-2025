// Compare geometry only on the sealed source grid. No solver or authority.
#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <iomanip>
#include <stdexcept>
namespace m = multi_purpose_mpc_ros;
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  std::string detail;
  auto record = m::mpcc_architecture_snapshot::load_recorded_interaction_snapshot(argv[1], &detail);
  if (!record) throw std::runtime_error(detail);
  const auto & source = record->source;
  if (!source.replay_world) return 3;
  const auto & world = *source.replay_world;
  const auto footprint = m::mpcc_rate_resolved_physical_wall::resolve_clearance_footprint(
    world.physical_footprint, world.hard_wall_clearance_m);
  if (!footprint || !source.wall_grid) return 3;
  const auto comparison = YAML::LoadFile(argv[2]);
  std::cout << std::setprecision(17) << "[\n";
  bool first = true;
  for (double horizon : {0.05, 0.13, 2.0}) {
    for (const auto name : {"A", "B", "C", "D"}) {
      for (bool hard_reserve : {false, true}) {
        std::vector<m::recovery_footprint::Pose2D> path;
        std::vector<double> times;
        for (const auto & row : comparison["rows"]) {
          const double t = row["elapsed_sec"].as<double>();
          if (t > horizon) continue;
          const auto pose = row["poses"][name];
          path.push_back({pose[0].as<double>(), pose[1].as<double>(), pose[2].as<double>()});
          times.push_back(t);
        }
        const auto result = m::recovery_footprint::evaluate_clear_footprint_path(
          *source.wall_grid, hard_reserve ? *footprint : world.physical_footprint, path, world.swept_step_m);
        if (!first) std::cout << ",\n";
        first = false;
        double rejected_sec = -1.;
        if (!result.clear && result.rejected_pose_available && result.rejected_path_index < times.size()) {
          const auto i = result.rejected_path_index;
          rejected_sec = i == 0 ? times[0] : times[i-1] + result.rejected_segment_ratio * (times[i]-times[i-1]);
        }
        std::cout << "{\"arm\":\"" << name << "\",\"horizon_limit_sec\":" << horizon
          << ",\"last_sample_sec\":" << times.back() << ",\"hard_reserve\":" << (hard_reserve ? "true" : "false")
          << ",\"valid\":" << (result.valid ? "true" : "false") << ",\"clear\":" << (result.clear ? "true" : "false")
          << ",\"rejected_path_index\":" << result.rejected_path_index << ",\"rejected_sec\":" << rejected_sec << "}";
      }
    }
  }
  std::cout << "\n]\n";
}
