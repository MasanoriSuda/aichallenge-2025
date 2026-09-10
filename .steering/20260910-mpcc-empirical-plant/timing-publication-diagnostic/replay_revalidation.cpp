#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main
#include <chrono>
#include "stop_recursion.hpp"

namespace model = m::mpcc_vehicle_model;
using Clock = std::chrono::steady_clock;

int main(int argc, char ** argv)
{
  if (argc != 3 && argc != 4) return 2;
  const std::filesystem::path path = argv[1];
  const auto root = YAML::LoadFile(path.string());
  YAML::Node out;
  out["authority"] = false;
  out["solver_invocations"] = 0;
  out["input"] = path.string();
  for (const auto key : {"previous_accepted_revalidation_evidence", "revalidation_evidence"}) {
    const auto e = root[key];
    if (e && e["status"]) {
      out[key]["capture_status"] = e["status"].as<std::string>();
      out[key]["capture_reason"] = e["reason"].as<std::string>();
      if (e["status"].as<std::string>() != "present")
        out[key]["comparison_role"] = "unpaired historical input; not the accepted immediate predecessor";
    }
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
    bool recovered_projection = false;
    const double captured_progress = r.control_origin_physical_progress_m;
    for (const auto & knot : knots(root["source"]["wall_course_frame_knots"])) {
      const double original = knot.progress_m + std::cos(knot.heading_rad)*(r.control_pose.x_m-knot.x_m) +
        std::sin(knot.heading_rad)*(r.control_pose.y_m-knot.y_m);
      if (std::abs(original-captured_progress)>1e-9) continue;
      if (recovered_projection) throw std::runtime_error("ambiguous Stop projection basis");
      recovered_projection = true;
      r.control_origin_physical_progress_m = captured_progress +
        std::cos(knot.heading_rad)*(s.x_m-r.control_pose.x_m) +
        std::sin(knot.heading_rad)*(s.y_m-r.control_pose.y_m);
      row["stop_projection_waypoint"] = knot.waypoint;
    }
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
    row["stop_progress_projection"] = recovered_projection ?
      "exact captured live tangent basis recovered and body/progress rebound together" :
      "captured scalar anchor retained; native body projection, not exact live projection";
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
    if (argc == 4 && std::string(key) == "revalidation_evidence") {
      // Explicitly false historical epoch, diagnostic only: isolate the
      // previously omitted decision-to-publication interval. Never write it
      // back to a recorded snapshot or use it as a production input.
      auto hypothetical = mpcc_observation::read_request(e["request"], path.parent_path());
      hypothetical.plan = read_plan(e["certified_plan_evidence"], path);
      auto observation = hypothetical.publication_prefix->observation;
      const double nominal = std::stod(argv[3]);
      auto diagnostic = row["backdated_publication_counterfactual"];
      diagnostic["authority"] = false;
      diagnostic["meaning"] = "counterfactual emission at decision time; real history remains unchanged";
      diagnostic["proposal"] = "original artifact packet; separate from the immediate Stop proposal above";
      if (observation.commands.size() < 2 || nominal <= observation.commands[observation.commands.size()-2].published_sec ||
        nominal >= observation.commands.back().published_sec || observation.commands.back().wire_acceleration_mps2 != -3)
        throw std::runtime_error("counterfactual must only move the last recorded brake within its predecessor interval");
      diagnostic["recorded_publication_sec"] = observation.commands.back().published_sec;
      diagnostic["counterfactual_publication_sec"] = nominal;
      observation.commands.back().published_sec = nominal;
      const auto rebound = model::predict_prospective_publication(
        observation, hypothetical.publication_prefix->proposed_packet, parameters);
      if (!rebound) throw std::runtime_error("counterfactual prefix rejected");
      // Recover the exact live projection basis only if it reproduces the
      // captured scalar. This is the controller's s + tangent dot displacement.
      const auto original_pose = hypothetical.control_pose;
      const double original_progress = hypothetical.control_origin_physical_progress_m;
      bool projection_recovered = false;
      for (const auto & knot : knots(root["source"]["wall_course_frame_knots"])) {
        const double reconstructed = knot.progress_m +
          std::cos(knot.heading_rad) * (original_pose.x_m-knot.x_m) +
          std::sin(knot.heading_rad) * (original_pose.y_m-knot.y_m);
        if (std::abs(reconstructed-original_progress) > 1e-9) continue;
        if (projection_recovered) throw std::runtime_error("ambiguous projection basis");
        projection_recovered = true;
        hypothetical.control_origin_physical_progress_m +=
          std::cos(knot.heading_rad) * (rebound->control_origin.x_m-original_pose.x_m) +
          std::sin(knot.heading_rad) * (rebound->control_origin.y_m-original_pose.y_m);
        diagnostic["projection_waypoint"] = knot.waypoint;
      }
      if (!projection_recovered) throw std::runtime_error("live projection basis unavailable");
      diagnostic["control_origin_physical_progress_m"] = hypothetical.control_origin_physical_progress_m;
      hypothetical.publication_prefix = rebound;
      const auto & origin = rebound->control_origin;
      hypothetical.control_pose = {origin.x_m, origin.y_m, origin.yaw_rad};
      hypothetical.current_speed_mps = rebound->current.forward_velocity_mps;
      hypothetical.control_origin_speed_mps = origin.forward_velocity_mps;
      hypothetical.current_time_steering_rad = rebound->current.tire_steering_rad;
      hypothetical.current_steering_rad = origin.desired_steering_rad;
      hypothetical.current_response_steering_rad = origin.tire_steering_rad;
      hypothetical.current_lateral_velocity_mps = origin.lateral_velocity_mps;
      hypothetical.current_yaw_rate_radps = origin.yaw_rate_radps;
      hypothetical.measured_to_control_path.clear();
      hypothetical.measured_to_control_elapsed_sec.clear();
      for (const auto & item : rebound->current_to_control) {
        hypothetical.measured_to_control_path.push_back({item.state.x_m, item.state.y_m, item.state.yaw_rad});
        hypothetical.measured_to_control_elapsed_sec.push_back(item.source_sec-hypothetical.now_sec);
      }
      const auto normal = retained::evaluate(hypothetical);
      const auto stop = retained::evaluate_published_stop_successor(hypothetical);
      diagnostic["current_speed_mps"] = hypothetical.current_speed_mps;
      diagnostic["control_origin_speed_mps"] = hypothetical.control_origin_speed_mps;
      diagnostic["control_x_m"] = origin.x_m;
      diagnostic["control_y_m"] = origin.y_m;
      diagnostic["control_yaw_rad"] = origin.yaw_rad;
      diagnostic["retained"] = retained::to_string(normal.reason);
      diagnostic["stop"] = retained::to_string(stop.result.reason);
      diagnostic["stop_wall_clear"] = stop.result.successor_path_clearance.clear;
      diagnostic["stop_samples"] = stop.result.actuation_samples.size();
    }
    if (built.plan) {
      r.plan = built.plan;
      r.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
      const auto join_start = Clock::now();
      const auto joined = retained::evaluate(r);
      row["join_elapsed_ms"] = std::chrono::duration<double, std::milli>(Clock::now() - join_start).count();
      row["join"] = retained::to_string(joined.reason);
      row["materialized_input_schema"] = r.plan->execution_artifact->identity.source_context.input_schema_id;
      row["materialized_model_closure"] = sample_stop_recursion(r, path.string());
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
      for (const auto name : {"requested-intent", "published-upstream-intent", "production-published-stop"}) {
        braking.current_intent = std::string(name) == "published-upstream-intent" ? upstream : r.current_intent;
        auto diagnostic = row["actual_published_stop_diagnostic"][name];
        diagnostic["authority"] = false;
        diagnostic["source"] = braking.plan->execution_artifact->identity.sequence;
        diagnostic["intent"] = m::mpcc_execution_contract::to_string(braking.current_intent);
        const auto prepared = std::string(name) == "production-published-stop" ?
          retained::evaluate_published_stop_successor(braking) :
          retained::PublishedStopSuccessorEvaluation{braking, retained::evaluate_stop_successor(braking)};
        const auto & checked = prepared.result;
        diagnostic["stop"] = retained::to_string(checked.reason);
        diagnostic["clearance"] = checked.dynamic_clearance.minimum_clearance_m;
        diagnostic["samples"] = checked.actuation_samples.size();
        if (!prepared.request) continue;
        diagnostic["proof_intent"] = m::mpcc_execution_contract::to_string(prepared.request->current_intent);
        const auto candidate = bundle::build(*prepared.request, checked, 200000U + r.decision_id);
        diagnostic["bundle"] = bundle::to_string(candidate.reason);
        if (candidate.plan) {
          auto join = *prepared.request;
          join.plan = candidate.plan;
          join.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
          const auto result = retained::evaluate(join);
          diagnostic["join"] = retained::to_string(result.reason);
          const auto production = m::mpcc_rate_resolved_production_adapter::build(result);
          diagnostic["production"] = m::mpcc_rate_resolved_production_adapter::to_string(production.reason);
          if (production.authority) {
            const auto & command = production.authority->command;
            if (!join.plan->execution_artifact->terminal_body_rest_required ||
              !join.publication_prefix || !model::publication_packet_matches(
                *join.publication_prefix, join.now_sec, command.acceleration_mps2,
                command.steering_tire_angle_rad, join.plan->execution_artifact->vehicle_model.steering_wire_gain))
              throw std::runtime_error("published Stop lost rest or packet binding");
            diagnostic["packet_acceleration"] = command.acceleration_mps2;
            diagnostic["packet_steering"] = command.steering_tire_angle_rad;
          }
        }
      }
    }
  }
  YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter << out;
  std::ofstream(argv[2]) << emitter.c_str() << '\n';
  std::cout << emitter.c_str() << '\n';
}
