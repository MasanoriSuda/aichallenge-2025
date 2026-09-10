// Observation only: reconstruct the node's current-world request for the exact
// actually published artifact, keeping the captured observation and world.
#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_applied_program.hpp"
#include "multi_purpose_mpc_ros/mpcc_wire_command.hpp"
#include <chrono>
namespace vehicle = m::mpcc_vehicle_model;

retained::Result report_actual(YAML::Node row, const retained::Request &request)
{
  const auto start = std::chrono::steady_clock::now();
  const auto result = retained::evaluate(request);
  row["elapsed_ms"] = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-start).count();
  row["source"] = request.plan->execution_artifact->identity.sequence;
  row["reason"] = retained::to_string(result.reason);
  row["applied_reason"] = static_cast<int>(result.applied_program_reason);
  row["program_reason"] = static_cast<int>(result.applied_program_prepare_reason);
  row["prediction_reason"] = static_cast<int>(result.applied_input_prediction_reason);
  row["rejected_sec"] = result.applied_program_rejected_sec;
  row["rejected_peer"] = result.applied_program_rejected_peer_id;
  row["terminal_attempts"] = result.terminal_stop_reference_attempts;
  const auto output = m::mpcc_rate_resolved_production_adapter::build(result);
  row["production"] = m::mpcc_rate_resolved_production_adapter::to_string(output.reason);
  if (request.publication_prefix) {
    const auto &p = request.publication_prefix->proposed_packet;
    row["packet"].push_back(p.published_sec);row["packet"].push_back(p.wire_acceleration_mps2);row["packet"].push_back(p.wire_steering_rad);
  }
  if (result.proof && result.proof->applied_program) {
    row["rest_sec"] = result.proof->applied_program->tube().rest_sec;
    row["peer_clearance"] = result.proof->applied_program->minimum_peer_clearance_m();
  }
  return result;
}

int main(int argc, char **argv)
{
  if (argc != 3) return 2;
  const std::filesystem::path path = argv[1];
  const auto root = YAML::LoadFile(path.string());
  const auto e = root["revalidation_evidence"], published = root["publication_bundle"];
  const auto original = mpcc_observation::read_request(e["request"], path.parent_path());
  if (original.follow_target) throw std::runtime_error("Follow rebind needs its original projection");
  const auto observation = vehicle::decode_observation_provenance(root["source"]["semantic_request"]["observation_provenance"]);
  if (!observation || observation->now_sec != original.now_sec || observation->control_origin_sec != original.control_origin_sec)
    throw std::runtime_error("current source observation unavailable");
  const auto plan = read_plan(published["certified_plan_evidence"], path, published["source"]);
  const auto publication = published["certified_plan_evidence"]["publication"];
  if (publication["failure_decision_id"].as<uint64_t>() != original.decision_id ||
      publication["failure_observation_sec"].as<double>() != original.now_sec ||
      publication["publication_decision_id"].as<uint64_t>() >= original.decision_id)
    throw std::runtime_error("publication association invalid");
  const retained::ExecutionClock clock{retained::ExecutionClockKind::PublishedPlan,
    publication["publication_control_origin_sec"].as<double>(), publication["publication_artifact_elapsed_sec"].as<double>()};
  const auto frames = knots(root["source"]["wall_course_frame_knots"]);
  const auto frame = std::find_if(frames.begin(), frames.end(), [&](const auto &k) {
    const double progress = k.progress_m + std::cos(k.heading_rad)*(original.control_pose.x_m-k.x_m) +
      std::sin(k.heading_rad)*(original.control_pose.y_m-k.y_m);
    return std::abs(progress-original.control_origin_physical_progress_m) <= 1e-9;
  });
  if (frame == frames.end()) throw std::runtime_error("original course association unavailable");
  const auto rebind = [&](auto selected, retained::ExecutionClock selected_clock, bool brake) {
    auto r = original; r.plan = selected; r.execution_clock = selected_clock;
    r.current_intent = selected->execution_artifact->identity.source_context.intent;
    r.publication_prefix.reset();
    const auto &model = selected->execution_artifact->vehicle_model;
    const auto packet = brake ? std::optional<vehicle::PublishedCommand>{{r.now_sec,
      static_cast<float>(r.minimum_acceleration_mps2),
      m::mpcc_wire_command::steering(r.previous_published_steering_rad, model.steering_wire_gain)}} :
      retained::prospective_artifact_packet(r);
    if (!packet) throw std::runtime_error("prospective packet unavailable");
    const auto predicted = vehicle::predict_prospective_publication(*observation, *packet, model);
    if (!predicted) throw std::runtime_error("prospective prefix unavailable");
    r.publication_prefix = predicted;
    const auto &state = predicted->control_origin;
    r.control_origin_physical_progress_m = frame->progress_m + std::cos(frame->heading_rad)*(state.x_m-frame->x_m) + std::sin(frame->heading_rad)*(state.y_m-frame->y_m);
    r.control_pose = {state.x_m,state.y_m,state.yaw_rad};
    r.current_speed_mps = predicted->current.forward_velocity_mps;
    r.control_origin_speed_mps = state.forward_velocity_mps;
    r.current_time_steering_rad = predicted->current.tire_steering_rad;
    r.current_steering_rad = state.desired_steering_rad;
    r.current_response_steering_rad = state.tire_steering_rad;
    r.current_lateral_velocity_mps = state.lateral_velocity_mps;
    r.current_yaw_rate_radps = state.yaw_rate_radps;
    r.measured_to_control_path.clear();r.measured_to_control_elapsed_sec.clear();
    for (const auto &s : predicted->current_to_control) {
      r.measured_to_control_path.push_back({s.state.x_m,s.state.y_m,s.state.yaw_rad});
      r.measured_to_control_elapsed_sec.push_back(s.source_sec-r.now_sec);
    }
    return r;
  };
  YAML::Node out;
  out["authority"] = false;out["decision"] = original.decision_id;
  out["meaning"] = "Original current source observation, immutable world, actual publication artifact/clock; explicitly reconstructed node-equivalent prospective requests. Not the original inspected-plan request.";
  const auto normal = rebind(plan,clock,false);
  const auto accepted_normal = report_actual(out["actual_published_normal"],normal);
  if (accepted_normal.proof && accepted_normal.proof->applied_program) {
    const auto stop = bundle::build_certified_terminal(normal, accepted_normal, 200000U+original.decision_id);
    out["normal_materialization"] = bundle::to_string(stop.reason);
    if (stop.plan) {
      const auto joined = rebind(stop.plan, {retained::ExecutionClockKind::TimeAlignedCandidate,NAN,NAN}, false);
      report_actual(out["normal_materialized_join"],joined);
    }
  }
  const auto braking = rebind(plan,clock,true);
  const auto prepared = retained::evaluate_published_stop_successor(braking);
  out["published_stop"] = retained::to_string(prepared.result.reason);
  if (prepared.request && prepared.result.accepted()) {
    const auto built = bundle::build(*prepared.request,prepared.result,100000U+original.decision_id);
    out["bundle"] = bundle::to_string(built.reason);
    if (built.plan) {
      const auto joined = rebind(built.plan,{retained::ExecutionClockKind::TimeAlignedCandidate,NAN,NAN},false);
      report_actual(out["actual_stop_join"],joined);
    }
  }
  YAML::Emitter emitter;emitter.SetDoublePrecision(17);emitter<<out;
  std::ofstream(argv[2])<<emitter.c_str()<<'\n';
}
