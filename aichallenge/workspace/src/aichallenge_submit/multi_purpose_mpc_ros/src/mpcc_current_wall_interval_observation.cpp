#include "multi_purpose_mpc_ros/mpcc_current_wall_interval_observation.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>

namespace multi_purpose_mpc_ros::mpcc_current_wall_interval_observation
{

void capture_result(
  Observation & o, const recovery_footprint::LateralClearRunsResult & runs,
  const recovery_footprint::LateralClearIntervalResult & selected) noexcept
{
  o.runs_valid = runs.valid;
  o.checked_pose_count = runs.checked_pose_count;
  o.actual_run_count = runs.clear_runs.size();
  std::copy_n(runs.clear_runs.begin(), std::min(o.actual_run_count, kMaximumClearRuns),
    o.clear_runs.begin());
  o.selected = selected;
}

mpcc_architecture_snapshot::RecordResult record(const Observation & o) noexcept
{
  namespace archive = mpcc_architecture_snapshot;
  archive::RecordResult result;
  if (!o.decision_id || !o.prior_moving_decision_id ||
    o.prior_moving_decision_id >= o.decision_id || !std::isfinite(o.ros_sec) ||
    !o.grid || !o.grid->valid() || !o.footprint.valid() || o.output_root.empty())
  {
    result.detail = "invalid current wall interval observation";
    return result;
  }
  try {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex);
    std::ostringstream name;
    name << "decision-" << std::setw(12) << std::setfill('0') << o.decision_id;
    const auto final = o.output_root / name.str();
    const auto temporary = o.output_root / (name.str() + ".tmp");
    result.snapshot_file = final / "snapshot.yaml";
    if (std::filesystem::exists(final)) {
      result.status = archive::RecordStatus::Duplicate;
      return result;
    }
    std::filesystem::create_directories(o.output_root);
    if (!std::filesystem::create_directory(temporary)) {
      throw std::runtime_error("previous current wall observation temporary directory exists");
    }
    YAML::Node root;
    root["schema"] = "mpcc-current-wall-interval-observation/v1";
    root["authority"] = false;
    root["complete"] = o.actual_run_count <= kMaximumClearRuns;
    root["decision_id"] = o.decision_id;
    root["ros_sec"] = o.ros_sec;
    root["prior_moving_decision_id"] = o.prior_moving_decision_id;
    root["prior_moving_pose_sec"] = o.prior_moving_pose_sec;
    root["prior_moving_velocity_mps"] = o.prior_moving_velocity_mps;
    root["intent"] = o.intent;
    root["waypoint"] = o.waypoint;
    const auto pose = [](const recovery_footprint::Pose2D & p) {
        return std::vector<double>{p.x_m, p.y_m, p.yaw_rad};
      };
    root["temporal_pose"] = pose(o.temporal_pose);
    root["reference_pose"] = pose(o.reference_pose);
    root["query_pose"] = pose(o.query_pose);
    root["projection_lag_lateral_heading"] = std::vector<double>{
      o.projected_lag_m, o.projected_lateral_m, o.projected_heading_rad};
    root["applied_lag_m"] = o.applied_lag_m;
    root["spatial_lateral_heading"] = std::vector<double>{o.spatial_lateral_m, o.spatial_heading_rad};
    root["bounds"] = std::vector<double>{o.lower_m, o.upper_m};
    root["clearance_m"] = o.clearance_m;
    root["sample_step_m"] = o.sample_step_m;
    root["boundary_guard_m"] = o.boundary_guard_m;
    root["sampling_anchor"] = std::vector<double>{
      o.sampling_anchor.lower_lateral_offset_m, o.sampling_anchor.upper_lateral_offset_m};
    root["footprint"] = std::vector<double>{o.footprint.front_extent_m, o.footprint.rear_extent_m,
      o.footprint.left_extent_m, o.footprint.right_extent_m, o.footprint.margin_m};
    auto grid = root["grid"];
    grid["width"] = o.grid->width; grid["height"] = o.grid->height;
    grid["resolution_m"] = o.grid->resolution_m;
    grid["origin_x_m"] = o.grid->origin_x_m; grid["origin_y_m"] = o.grid->origin_y_m;
    grid["y_axis"] = static_cast<int>(o.grid->y_axis);
    grid["fingerprint"] = recovery_footprint::occupancy_grid_fingerprint(*o.grid);
    grid["payload"] = "wall-grid.bin";
    root["runs_valid"] = o.runs_valid;
    root["checked_pose_count"] = o.checked_pose_count;
    root["actual_run_count"] = o.actual_run_count;
    root["clear_runs"] = YAML::Node(YAML::NodeType::Sequence);
    for (std::size_t i = 0; i < std::min(o.actual_run_count, kMaximumClearRuns); ++i) {
      root["clear_runs"].push_back(std::vector<double>{
        o.clear_runs[i].lower_lateral_offset_m, o.clear_runs[i].upper_lateral_offset_m});
    }
    auto selected = root["selected"];
    selected["valid"] = o.selected.valid;
    selected["feasible"] = o.selected.feasible;
    selected["preferred_lateral_contained"] = o.selected.preferred_lateral_contained;
    selected["lower_m"] = o.selected.lower_lateral_offset_m;
    selected["upper_m"] = o.selected.upper_lateral_offset_m;
    selected["checked_pose_count"] = o.selected.checked_pose_count;
    std::ofstream cells(temporary / "wall-grid.bin", std::ios::binary);
    for (const auto cell : o.grid->cells) {
      const auto byte = static_cast<std::int8_t>(cell);
      cells.write(reinterpret_cast<const char *>(&byte), sizeof(byte));
    }
    cells.close();
    if (!cells) throw std::runtime_error("cannot write current wall observation grid");
    YAML::Emitter emitter; emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
    emitter << root;
    if (!emitter.good()) throw std::runtime_error("cannot serialize current wall observation");
    std::ofstream yaml(temporary / "snapshot.yaml"); yaml << emitter.c_str() << '\n'; yaml.close();
    if (!yaml) throw std::runtime_error("cannot write current wall observation");
    std::filesystem::rename(temporary, final);
    result.status = archive::RecordStatus::Written;
  } catch (const std::exception & error) {
    result.status = archive::RecordStatus::IoFailure; result.detail = error.what();
  } catch (...) {
    result.status = archive::RecordStatus::IoFailure;
    result.detail = "unknown current wall observation failure";
  }
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_current_wall_interval_observation
