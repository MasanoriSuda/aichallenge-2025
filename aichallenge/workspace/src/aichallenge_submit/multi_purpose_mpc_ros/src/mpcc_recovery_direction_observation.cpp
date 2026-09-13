#include "multi_purpose_mpc_ros/mpcc_recovery_direction_observation.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>

namespace multi_purpose_mpc_ros::mpcc_recovery_direction_observation
{

void append_trial(
  Observation & o, const char * phase, recovery_footprint::ReversePrimitive primitive,
  const recovery_footprint::ReverseRolloutParameters & parameters,
  recovery_footprint::ContactEscapePolicy policy, double minimum_reduction,
  const recovery_footprint::FeasibilityResult & result) noexcept
{
  ++o.attempted_trial_count;
  if (!phase || o.trials.size() >= kMaximumTrials) {
    o.complete = false;
    return;
  }
  try {
    Trial trial;
    trial.phase = phase;
    trial.primitive = primitive;
    trial.rollout = parameters;
    trial.contact_policy = policy;
    trial.minimum_contact_reduction_ratio = minimum_reduction;
    auto & saved = trial.result_without_rollout;
    saved.feasible = result.feasible;
    saved.reason = result.reason;
    saved.primitive = result.primitive;
    saved.initial_contact_count = result.initial_contact_count;
    saved.maximum_contact_count = result.maximum_contact_count;
    saved.final_contact_count = result.final_contact_count;
    saved.checked_pose_count = result.checked_pose_count;
    saved.contact_reduction = result.contact_reduction;
    saved.steering_angle_rad = result.steering_angle_rad;
    saved.rejected_at_distance_m = result.rejected_at_distance_m;
    trial.rollout_pose_count = result.rollout.size();
    if (!result.rollout.empty()) {
      trial.endpoint = result.rollout.back().pose;
      trial.course = stuck_recovery::resolve_recovery_course_progress({
        o.lateral_error_m, o.pose.yaw_rad, o.heading_error_rad,
        trial.endpoint->x_m - o.pose.x_m, trial.endpoint->y_m - o.pose.y_m,
        o.course_activation_lateral_m, o.course_worsening_tolerance_m});
    }
    o.trials.push_back(std::move(trial));
  } catch (...) {
    o.complete = false;
  }
}

mpcc_architecture_snapshot::RecordResult record(const Observation & o) noexcept
{
  namespace archive = mpcc_architecture_snapshot;
  archive::RecordResult result;
  if (!o.decision_id || !o.grid || !o.grid->valid() || !o.footprint.valid() ||
    o.output_root.empty() || !std::isfinite(o.ros_sec) || !std::isfinite(o.steady_sec) ||
    o.trials.size() > kMaximumTrials || o.attempted_trial_count < o.trials.size())
  {
    result.detail = "invalid Recovery direction observation";
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
      throw std::runtime_error("previous Recovery observation temporary directory exists");
    }
    YAML::Node root;
    root["schema"] = "mpcc-recovery-direction-observation/v1";
    root["authority"] = false;
    root["complete"] = o.complete && o.attempted_trial_count == o.trials.size();
    root["decision_id"] = o.decision_id;
    root["ros_sec"] = o.ros_sec;
    root["steady_sec"] = o.steady_sec;
    root["signed_speed_mps"] = o.signed_speed_mps;
    root["pose"] = std::vector<double>{o.pose.x_m, o.pose.y_m, o.pose.yaw_rad};
    root["footprint"] = std::vector<double>{o.footprint.front_extent_m, o.footprint.rear_extent_m,
      o.footprint.left_extent_m, o.footprint.right_extent_m, o.footprint.margin_m};
    root["course"] = std::vector<double>{o.lateral_error_m, o.heading_error_rad,
      o.course_activation_lateral_m, o.course_worsening_tolerance_m};
    root["reverse_only"] = o.reverse_only;
    root["forward_probe_allowed"] = o.forward_probe_allowed;
    root["prefer_forward"] = o.prefer_forward;
    root["full_rollout"] = o.full_rollout;
    root["current_footprint_clear"] = o.current_footprint_clear;
    root["wall_region"] = o.wall_region;
    root["selected_direction"] = o.selected_direction;
    root["static_clear"] = o.static_clear;
    root["v2x_clear"] = o.v2x_clear;
    root["information_complete"] = o.information_complete;
    root["attempted_trial_count"] = o.attempted_trial_count;
    auto grid = root["grid"];
    grid["width"] = o.grid->width; grid["height"] = o.grid->height;
    grid["resolution_m"] = o.grid->resolution_m;
    grid["origin_x_m"] = o.grid->origin_x_m; grid["origin_y_m"] = o.grid->origin_y_m;
    grid["y_axis"] = static_cast<int>(o.grid->y_axis);
    grid["fingerprint"] = recovery_footprint::occupancy_grid_fingerprint(*o.grid);
    grid["payload"] = "wall-grid.bin";
    for (const auto & t : o.trials) {
      YAML::Node row;
      row["phase"] = t.phase; row["primitive"] = static_cast<int>(t.primitive);
      row["rollout_parameters"] = std::vector<double>{t.rollout.reverse_distance_m,
        t.rollout.rollout_step_m, t.rollout.swept_step_m, t.rollout.wheelbase_m, t.rollout.steering_angle_rad};
      row["contact_policy"] = static_cast<int>(t.contact_policy);
      row["minimum_contact_reduction_ratio"] = t.minimum_contact_reduction_ratio;
      const auto & r = t.result_without_rollout;
      row["feasible"] = r.feasible; row["reason"] = static_cast<int>(r.reason);
      row["reason_text"] = recovery_footprint::to_string(r.reason);
      row["contact_counts"] = std::vector<std::size_t>{r.initial_contact_count,
        r.maximum_contact_count, r.final_contact_count, r.contact_reduction};
      row["checked_pose_count"] = r.checked_pose_count;
      row["steering_angle_rad"] = r.steering_angle_rad;
      row["rejected_at_distance_m"] = r.rejected_at_distance_m;
      row["rollout_pose_count"] = t.rollout_pose_count;
      if (t.endpoint) row["endpoint"] = std::vector<double>{t.endpoint->x_m, t.endpoint->y_m, t.endpoint->yaw_rad};
      row["course_preview"]["valid"] = t.course.valid;
      row["course_preview"]["guard_active"] = t.course.guard_active;
      row["course_preview"]["candidate_allowed"] = t.course.candidate_allowed;
      row["course_preview"]["candidate_lateral_error_m"] = t.course.candidate_lateral_error_m;
      row["course_preview"]["lateral_improvement_m"] = t.course.lateral_improvement_m;
      root["trials"].push_back(row);
    }
    std::ofstream cells(temporary / "wall-grid.bin", std::ios::binary);
    for (const auto cell : o.grid->cells) {
      const auto byte = static_cast<std::int8_t>(cell);
      cells.write(reinterpret_cast<const char *>(&byte), sizeof(byte));
    }
    cells.close();
    if (!cells) throw std::runtime_error("cannot write Recovery observation grid");
    YAML::Emitter emitter; emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
    emitter << root;
    if (!emitter.good()) throw std::runtime_error("cannot serialize Recovery observation");
    std::ofstream yaml(temporary / "snapshot.yaml"); yaml << emitter.c_str() << '\n'; yaml.close();
    if (!yaml) throw std::runtime_error("cannot write Recovery observation");
    std::filesystem::rename(temporary, final);
    result.status = archive::RecordStatus::Written;
  } catch (const std::exception & error) {
    result.status = archive::RecordStatus::IoFailure; result.detail = error.what();
  } catch (...) {
    result.status = archive::RecordStatus::IoFailure; result.detail = "unknown Recovery observation failure";
  }
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_recovery_direction_observation
