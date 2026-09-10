#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main
#include <chrono>

namespace model = m::mpcc_vehicle_model;
using Clock = std::chrono::steady_clock;

int main(int argc, char ** argv)
{
  if (argc != 3) return 2;
  const std::filesystem::path path = argv[1];
  const auto root = YAML::LoadFile(path.string());
  YAML::Node out;
  out["authority"] = false;
  out["solver_invocations"] = 0;
  out["input"] = path.string();
  for (const auto key : {"previous_accepted_revalidation_evidence", "revalidation_evidence"}) {
    const auto e = root[key];
    if (!e || !e["request"] || !e["certified_plan_evidence"]) {
      out[key]["status"] = "unavailable";
      out[key]["reason"] = e && e["reason"] ? e["reason"].as<std::string>() : "missing recorded request/plan";
      continue;
    }
    auto r = mpcc_observation::read_request(e["request"], path.parent_path());
    r.plan = read_plan(e["certified_plan_evidence"], path);
    auto row = out[key];
    row["decision"] = r.decision_id;
    row["source"] = r.plan->execution_artifact->identity.sequence;
    const auto started = Clock::now();
    const auto result = retained::evaluate(r);
    row["elapsed_ms"] = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
    row["reason"] = retained::to_string(result.reason);
    row["terminal_clearance"] = result.terminal_stop_minimum_dynamic_clearance_m;
    row["terminal_samples"] = result.proof ? result.proof->terminal_stop_actuation_samples.size() : 0;
    const auto & parameters = r.plan->execution_artifact->vehicle_model;
    const auto publication = m::mpcc_rate_resolved_production_adapter::build(result);
    row["production"] = m::mpcc_rate_resolved_production_adapter::to_string(publication.reason);
    if (result.reason == retained::Reason::Accepted && !publication.authority)
      throw std::runtime_error("accepted revalidation did not produce matching authority");
    if (publication.authority) {
      const auto & command = publication.authority->command;
      row["packet_acceleration"] = command.acceleration_mps2;
      row["packet_steering"] = command.steering_tire_angle_rad;
      if (!r.publication_prefix || !model::publication_packet_matches(*r.publication_prefix,
          r.now_sec, command.acceleration_mps2, command.steering_tire_angle_rad,
          parameters.steering_wire_gain))
        throw std::runtime_error("production packet differs from its prospective proof");
    }
    if (!r.publication_prefix) throw std::runtime_error("missing prospective original observation");
    const auto observation = r.publication_prefix->observation;
    const auto prefix = model::predict_prospective_publication(observation,
      {r.now_sec, r.minimum_acceleration_mps2,
        static_cast<float>(r.previous_published_steering_rad * parameters.steering_wire_gain)}, parameters);
    if (!prefix) throw std::runtime_error("invalid Stop proposal");
    const auto & s = prefix->control_origin;
    r.publication_prefix = prefix;
    r.control_pose = {s.x_m, s.y_m, s.yaw_rad};
    r.current_speed_mps = prefix->current.forward_velocity_mps;
    r.control_origin_speed_mps = s.forward_velocity_mps;
    r.current_time_steering_rad = prefix->current.tire_steering_rad;
    r.current_steering_rad = s.desired_steering_rad;
    r.current_response_steering_rad = s.tire_steering_rad;
    r.current_lateral_velocity_mps = s.lateral_velocity_mps;
    r.current_yaw_rate_radps = s.yaw_rate_radps;
    r.measured_to_control_path.clear();
    r.measured_to_control_elapsed_sec.clear();
    for (const auto & item : prefix->current_to_control) {
      r.measured_to_control_path.push_back({item.state.x_m, item.state.y_m, item.state.yaw_rad});
      r.measured_to_control_elapsed_sec.push_back(item.source_sec - r.now_sec);
    }
    // This diagnostic keeps the captured scalar progress anchor. Native
    // continuation projects the new body; it is not a live waypoint replay.
    row["stop_progress_projection"] = "captured scalar anchor retained; native body projection";
    const auto stop_start = Clock::now();
    const auto stop = retained::evaluate_stop_successor(r);
    row["stop_elapsed_ms"] = std::chrono::duration<double, std::milli>(Clock::now() - stop_start).count();
    row["stop"] = retained::to_string(stop.reason);
    row["stop_clearance"] = stop.dynamic_clearance.minimum_clearance_m;
    row["stop_samples"] = stop.actuation_samples.size();
    const auto build_start = Clock::now();
    const auto built = bundle::build(r, stop, 100000U + r.decision_id);
    row["build_elapsed_ms"] = std::chrono::duration<double, std::milli>(Clock::now() - build_start).count();
    row["bundle"] = bundle::to_string(built.reason);
    if (built.plan) {
      r.plan = built.plan;
      r.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
      const auto join_start = Clock::now();
      const auto joined = retained::evaluate(r);
      row["join_elapsed_ms"] = std::chrono::duration<double, std::milli>(Clock::now() - join_start).count();
      row["join"] = retained::to_string(joined.reason);
    }
    const auto published = root["publication_bundle"]["certified_plan_evidence"];
    if (std::string(key) == "revalidation_evidence" && published && published["publication"]) {
      // A proposed intent may differ from the last actual publication. This
      // counterfactual tests only that published source's full Stop obligation;
      // it never authorizes continuing an expired tactical maneuver.
      auto braking = r;
      braking.plan = read_plan(published, path);
      const auto publication = published["publication"];
      braking.execution_clock = {retained::ExecutionClockKind::PublishedPlan,
        publication["publication_control_origin_sec"].as<double>(),
        publication["publication_artifact_elapsed_sec"].as<double>()};
      const auto upstream = braking.plan->execution_artifact->identity.source_context.intent;
      for (const auto name : {"requested-intent", "published-upstream-intent"}) {
        braking.current_intent = std::string(name) == "requested-intent" ? r.current_intent : upstream;
        auto diagnostic = row["actual_published_stop_diagnostic"][name];
        diagnostic["authority"] = false;
        diagnostic["source"] = braking.plan->execution_artifact->identity.sequence;
        diagnostic["intent"] = m::mpcc_execution_contract::to_string(braking.current_intent);
        const auto checked = retained::evaluate_stop_successor(braking);
        diagnostic["stop"] = retained::to_string(checked.reason);
        diagnostic["clearance"] = checked.dynamic_clearance.minimum_clearance_m;
        diagnostic["samples"] = checked.actuation_samples.size();
        const auto candidate = bundle::build(braking, checked, 200000U + r.decision_id);
        diagnostic["bundle"] = bundle::to_string(candidate.reason);
        if (candidate.plan) {
          auto join = braking;
          join.plan = candidate.plan;
          join.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
          const auto result = retained::evaluate(join);
          diagnostic["join"] = retained::to_string(result.reason);
          diagnostic["production"] = m::mpcc_rate_resolved_production_adapter::to_string(
            m::mpcc_rate_resolved_production_adapter::build(result).reason);
        }
      }
    }
  }
  YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter << out;
  std::ofstream(argv[2]) << emitter.c_str() << '\n';
  std::cout << emitter.c_str() << '\n';
}
