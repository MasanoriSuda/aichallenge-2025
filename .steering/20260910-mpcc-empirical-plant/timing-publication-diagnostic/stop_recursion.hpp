#pragma once
#include <chrono>
namespace adapter = m::mpcc_rate_resolved_physical_adapter;

inline YAML::Node sample_stop_recursion(retained::Request captured, const std::string & input)
{
  const auto & a = *captured.plan->execution_artifact;
  if (!a.terminal_body_rest_required) throw std::runtime_error("requires published complete Stop");
  const auto & initial = a.semantic_initial_state.value();
  const auto full = adapter::build_continuation(a,
    artifact::resolve_cursor(a, a.prediction_origin_sec),
    {initial.lateral_m, initial.lag_m, initial.heading_offset_rad,
     initial.velocity_mps, initial.progress_m, initial.steering_rad,
     initial.response_steering_rad, initial.lateral_velocity_mps, initial.yaw_rate_radps});
  if (!full.exact_trajectory) throw std::runtime_error("initial continuation unavailable");
  const auto & trajectory = *full.exact_trajectory;
  YAML::Node out;
  out["authority"] = false;
  out["input"] = input;
  out["source"] = a.identity.sequence;
  out["meaning"] = "model closure diagnostic: synthetic exact model body, zero observation delay, frozen captured world; no public-input acceptance";
  for (const double elapsed : {0., .025, .035, .05, .075, .1, .15, .2}) {
    if (elapsed >= trajectory.elapsed_time_sec.back()-1e-9) continue;
    auto r = captured;
    auto state = initial;
    if (elapsed > 0.) {
      const auto it = std::find_if(trajectory.elapsed_time_sec.begin(), trajectory.elapsed_time_sec.end(),
        [elapsed](double t) {return std::abs(t-elapsed)<1e-9;});
      if (it == trajectory.elapsed_time_sec.end()) throw std::runtime_error("exact sample unavailable");
      const auto i = std::distance(trajectory.elapsed_time_sec.begin(), it);
      const auto & sample = full.actuation_samples.at(i);
      state = {trajectory.lateral_m[i], trajectory.lag_m[i], trajectory.heading_offset_rad[i],
        trajectory.velocity_mps[i], trajectory.progress_m[i]-a.course_progress_origin_m,
        sample.end_steering_rad, sample.end_response_steering_rad,
        sample.end_lateral_velocity_mps, sample.end_yaw_rate_radps};
    }
    const double absolute = a.course_progress_origin_m + state.progress_m;
    const auto frame = m::mpc_stage_geometry::sample_course_frame(
      captured.plan->physical_snapshot->course_frame_knots, absolute);
    if (!frame) throw std::runtime_error("frame unavailable");
    r.control_pose = {frame->x_m+std::cos(frame->heading_rad)*state.lag_m-std::sin(frame->heading_rad)*state.lateral_m,
      frame->y_m+std::sin(frame->heading_rad)*state.lag_m+std::cos(frame->heading_rad)*state.lateral_m,
      frame->heading_rad+state.heading_offset_rad};
    r.now_sec = r.control_origin_sec = a.prediction_origin_sec+elapsed;
    r.current_intent = a.identity.source_context.intent;
    r.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
    r.publication_prefix.reset();
    r.publication_prefix_required = false;
    r.current_speed_mps = r.control_origin_speed_mps = state.velocity_mps;
    r.current_time_steering_rad = r.current_response_steering_rad = state.response_steering_rad;
    r.current_steering_rad = state.steering_rad;
    r.current_lateral_velocity_mps = state.lateral_velocity_mps;
    r.current_yaw_rate_radps = state.yaw_rate_radps;
    r.control_origin_physical_progress_m = absolute+state.lag_m;
    const auto previous = artifact::extract_actuation(a, artifact::resolve_cursor(a,
      a.prediction_origin_sec+std::max(0.,elapsed-a.publication_interval_sec)));
    if (!previous.actuation) throw std::runtime_error("previous actuation unavailable");
    r.previous_published_steering_rad = previous.actuation->steering_rad;
    r.previous_published_command_age_sec = a.publication_interval_sec;
    r.measured_to_control_path = {r.control_pose};
    r.measured_to_control_elapsed_sec = {0.};
    // Keep the captured obstacle positions fixed, with the declared origin
    // moved together. This isolates ego suffix closure, not peer prediction.
    r.obstacles.observed_sec = r.now_sec;
    const auto started = std::chrono::steady_clock::now();
    const auto result = retained::evaluate(r);
    YAML::Node row;
    row["elapsed_sec"] = elapsed;
    row["reason"] = retained::to_string(result.reason);
    row["continuation"] = adapter::to_string(result.continuation_reason);
    row["wall_clear"] = result.continuation_path_clearance.clear;
    row["terminal_certified"] = result.terminal_stop_certified;
    row["uses_solved_suffix"] = result.terminal_stop_uses_solved_suffix;
    row["pose_error_m"] = result.control_pose_error_m;
    row["command_stage"] = result.command_control_stage_index;
    row["stage_advanced"] = result.publication_stage_advanced;
    row["speed_mps"] = state.velocity_mps;
    row["steering_rad"] = state.steering_rad;
    row["evaluation_ms"] = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    out["samples"].push_back(row);
  }
  return out;
}
