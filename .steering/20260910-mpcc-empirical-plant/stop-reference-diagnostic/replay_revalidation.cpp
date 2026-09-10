#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main
// Expose each existing reference hypothesis separately, with no production edit.
#include "mpcc_rate_resolved_retained_revalidation.cpp"

namespace adapter = m::mpcc_rate_resolved_physical_adapter;

int main(int argc, char ** argv)
{
  if (argc != 3) return 2;
  const std::filesystem::path path = argv[1], output = argv[2];
  std::filesystem::create_directory(output);
  const auto root = YAML::LoadFile(path.string());
  YAML::Node out;
  out["authority"] = false;
  out["solver_invocations"] = 0;
  for (const auto key : {"previous_accepted_revalidation_evidence", "revalidation_evidence"}) {
    const auto e = root[key];
    auto r = mpcc_observation::read_request(e["request"], path.parent_path());
    r.plan = read_plan(e["certified_plan_evidence"], path);
    const auto & a = *r.plan->execution_artifact;
    const auto profile = adapter::build_normal_path_stop_profile(a);
    if (!profile) throw std::runtime_error("missing original profile");
    for (const int variant : {0, 1, 2}) {
      const bool normal = variant != 1;
      const std::string label = variant == 0 ? "normal" : variant == 1 ? "track" : "bounded-constant-lateral-tail";
      auto reference = *profile;
      const auto & geometry = r.plan->physical_snapshot->terminal_stop_course_geometry;
      if (variant == 2 && geometry.progress_m.back() > reference.progress_m.back()) {
        reference.progress_m.push_back(geometry.progress_m.back());
        reference.lateral_m.push_back(reference.lateral_m.back());
      }
      auto row = out[key][label];
      row["reference_progress"] = reference.progress_m;
      row["reference_lateral"] = reference.lateral_m;
      row["meaning"] = variant == 2 ? "Diagnostic terminal reference candidate: unchanged solved lateral profile followed by an explicit constant-lateral course tail, bounded by sealed Stop geometry; source/world/packet unchanged, no runtime authority" : "existing production reference";
      const auto result = retained::evaluate_with_stop_profile(r, normal ? &reference : nullptr);
      row["decision"] = r.decision_id;
      row["reason"] = retained::to_string(result.reason);
      row["terminal"] = adapter::to_string(result.terminal_stop_reason);
      row["wall_clear"] = result.terminal_stop_path_clearance.clear;
      row["wall_checked"] = result.terminal_stop_path_clearance.checked_pose_count;
      row["final_steering"] = result.terminal_stop_final_steering_rad;
      if (!result.current_control_state_available) throw std::runtime_error("missing current state");
      const auto cursor = retained::resolve_execution_cursor(a, r.control_origin_sec, r.execution_clock);
      retained::Result diagnostic;
      const auto selected = retained::select_publication_actuation(r, cursor, diagnostic);
      if (!selected) throw std::runtime_error("missing source actuation");
      const auto & s = result.current_control_state;
      auto actuation = selected->second;
      actuation.predicted_speed_mps = r.control_origin_speed_mps;
      const auto stop = adapter::build_stop_contingency(a, selected->first, actuation,
        {s.lateral_m, s.lag_m, s.heading_offset_rad, s.velocity_mps, s.progress_m,
         actuation.steering_rad, r.current_response_steering_rad,
         r.current_lateral_velocity_mps, r.current_yaw_rate_radps},
        r.plan->physical_snapshot->terminal_stop_course_geometry, r.stop_lateral_policy,
        r.minimum_acceleration_mps2, 0.0, normal ? &reference : nullptr);
      row["direct_terminal"] = adapter::to_string(stop.reason);
      row["samples"] = stop.actuation_samples.size();
      if (stop.reason != result.terminal_stop_reason ||
          (stop.exact_trajectory && std::abs(stop.braking_suffix_final_steering_rad -
             result.terminal_stop_final_steering_rad) > 1e-9))
        throw std::runtime_error("separate native reconstruction differs from production");
      if (!stop.exact_trajectory) continue;
      const auto & t = *stop.exact_trajectory;
      const auto name = std::to_string(r.decision_id) + "-" + label + ".csv";
      std::ofstream csv(output / name);
      csv << std::setprecision(17) << "time,u,vy,r,steering,tire,ey,lag,epsi,progress,curvature,target_lateral,x,y,yaw\n";
      for (std::size_t i = 0; i < t.progress_m.size(); ++i) {
        const auto & sample = stop.actuation_samples[i];
        const auto frame = m::mpc_stage_geometry::sample_course_frame(
          r.plan->physical_snapshot->course_frame_knots, t.progress_m[i],
          std::max(1e-9, r.plan->physical_snapshot->bound_tolerance_m));
        if (!frame) throw std::runtime_error("missing course frame");
        const auto pose = contract::reconstruct_planar_pose_from_frenet(
          {frame->x_m, frame->y_m, frame->heading_rad},
          {t.lateral_m[i], t.lag_m[i], t.heading_offset_rad[i]});
        if (!pose) throw std::runtime_error("missing pose");
        csv << t.elapsed_time_sec[i] << ',' << t.velocity_mps[i] << ','
            << sample.end_lateral_velocity_mps << ',' << sample.end_yaw_rate_radps << ','
            << sample.end_steering_rad << ',' << sample.end_response_steering_rad << ','
            << t.lateral_m[i] << ',' << t.lag_m[i] << ',' << t.heading_offset_rad[i] << ','
            << t.progress_m[i] << ',' << sample.path_curvature_radpm << ','
            << (normal ? adapter::sample_stop_lateral_target(reference,
                t.progress_m[i] - a.course_progress_origin_m, a.physical_global_tolerance).value_or(NAN) : 0.0)
            << ',' << pose->x_m << ',' << pose->y_m << ',' << pose->yaw_rad << '\n';
      }
      row["trajectory"] = name;
    }
  }
  YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter << out;
  std::ofstream(output / "report.yaml") << emitter.c_str() << '\n';
  std::cout << emitter.c_str() << '\n';
}
