#define MPCC_WALL_AUDIT_NO_MAIN
#include "audit_applied_wall.cpp"

void probe_program(YAML::Node row, const retained::Request & world,
  const vehicle::PublishedInputProgram & program)
{
  const auto & observation = world.publication_prefix->observation;
  const auto & a = *world.plan->execution_artifact;
  const auto prediction = vehicle::predict_applied_inputs_to_rest(
    observation, program, *world.input_application_profile, a.vehicle_model);
  row["prediction_reason"] = static_cast<int>(prediction.reason);
  if (!prediction.tube) return;
  const auto footprint = physical::resolve_clearance_footprint(
    world.current_footprint, world.plan->physical_snapshot->hard_wall_clearance_m);
  if (!footprint || world.follow_target) throw std::runtime_error("unsupported diagnostic world");
  const auto to_world = [&](const rec::Pose2D & p) {
    const auto & o = observation.initial.state;
    const double c = std::cos(o.yaw_rad), s = std::sin(o.yaw_rad);
    return rec::Pose2D{o.x_m + c*p.x_m - s*p.y_m, o.y_m + s*p.x_m + c*p.y_m, o.yaw_rad + p.yaw_rad};
  };
  double first_wall = NAN, first_peer = NAN, minimum_peer = INFINITY;
  bool state_clear = true;
  const auto check = [&](const vehicle::BodyRanges & body, double begin, double end) {
    num::Box box;
    for (std::size_t i = 0; i < box.size(); ++i) box[i] = {body[i].lower, body[i].upper};
    const double tol = a.physical_global_tolerance;
    state_clear = state_clear && box[3].lo >= -tol &&
      std::max(std::abs(box[6].lo), std::abs(box[6].hi)) <= a.maximum_abs_steering_rad + tol &&
      std::max(std::abs(box[7].lo), std::abs(box[7].hi)) <= a.vehicle_model.maximum_wire_steering_rad*a.vehicle_model.tire_grip + tol;
    const auto f = num::footprint(box, *footprint);
    const auto cells = rec::sample_footprint(*world.current_wall_grid, f.extents, to_world(f.pose));
    if ((!cells.valid || cells.out_of_map || !cells.contact_cells.empty()) && !std::isfinite(first_wall)) first_wall = begin;
    const auto ego = num::footprint(box, world.current_footprint);
    for (const auto & obstacle : world.obstacles.obstacles) {
      auto circle = obstacle.circle;
      const double t0 = begin-world.obstacles.observed_sec, t1 = end-world.obstacles.observed_sec;
      circle.radius_m = num::up(circle.radius_m + num::up(circle.maximum_speed(t0,t1)*num::up((end-begin)/2)));
      const auto clearance = rec::circle_obstacle_clearance_at_time(ego.extents, to_world(ego.pose), circle, (t0+t1)/2);
      if (!clearance) throw std::runtime_error("invalid diagnostic peer");
      minimum_peer = std::min(minimum_peer, *clearance);
      if (*clearance < 0 && !std::isfinite(first_peer)) first_peer = begin;
    }
  };
  check(prediction.tube->publication_body, world.now_sec, world.now_sec);
  for (const auto & sample : prediction.tube->source_to_rest)
    if (sample.begin_sec >= world.now_sec) check(sample.swept_body, sample.begin_sec, sample.end_sec);
  row["rest_sec"] = prediction.tube->rest_sec;
  row["first_wall_sec"] = first_wall; row["first_peer_sec"] = first_peer;
  row["state_clear"] = state_clear; row["minimum_peer_m"] = minimum_peer;
  for (const auto & packet : program.commands) {
    YAML::Node p; p.push_back(packet.published_sec); p.push_back(packet.wire_acceleration_mps2);
    p.push_back(packet.wire_steering_rad); row["program"].push_back(p);
  }
}

int main(int argc, char ** argv)
{
  if (argc != 3) return 2;
  const std::filesystem::path path = argv[1];
  const auto root = YAML::LoadFile(path.string());
  const auto old = root["previous_accepted_revalidation_evidence"];
  auto previous = mpcc_observation::read_request(old["request"], path.parent_path());
  previous.plan = read_plan(old["certified_plan_evidence"], path);
  const auto published = read_plan(root["publication_bundle"]["certified_plan_evidence"], path);
  if (!artifact::same_identity(previous.plan->execution_artifact->identity, published->execution_artifact->identity))
    throw std::runtime_error("previous accepted source was not the actual publication source");
  const auto accepted = retained::evaluate(previous);
  if (!accepted.proof || !accepted.proof->applied_program)
    throw std::runtime_error("previous full applied proof unavailable");
  const auto materialized = bundle::build_certified_terminal(previous, accepted, 100000U + previous.decision_id);
  if (!materialized.plan) throw std::runtime_error("previous certified programme could not materialize");
  const auto captured = root["revalidation_evidence"];
  auto request = mpcc_observation::read_request(captured["request"], path.parent_path());
  auto original_request = request;
  original_request.plan = read_plan(captured["certified_plan_evidence"], path);
  const auto observation = request.publication_prefix->observation;
  const auto & model = materialized.plan->execution_artifact->vehicle_model;
  const auto history = vehicle::predict_published_history(observation.initial, observation.now_sec,
    observation.control_origin_sec, observation.commands, model,
    observation.acceleration_delay_sec, observation.steering_delay_sec);
  if (!history) throw std::runtime_error("original history model unavailable");
  const auto frames = knots(root["source"]["wall_course_frame_knots"]);
  const auto frame = std::find_if(frames.begin(), frames.end(), [&](const auto & k) {
    const double progress = k.progress_m + std::cos(k.heading_rad)*(request.control_pose.x_m-k.x_m) +
      std::sin(k.heading_rad)*(request.control_pose.y_m-k.y_m);
    return std::abs(progress-request.control_origin_physical_progress_m) <= 1e-9;
  });
  if (frame == frames.end()) throw std::runtime_error("original nominal course association unavailable");
  const auto set_state = [&](const vehicle::State & current, const vehicle::State & origin) {
    request.control_origin_physical_progress_m = frame->progress_m +
      std::cos(frame->heading_rad)*(origin.x_m-frame->x_m) + std::sin(frame->heading_rad)*(origin.y_m-frame->y_m);
    request.control_pose = {origin.x_m, origin.y_m, origin.yaw_rad};
    request.current_speed_mps = current.forward_velocity_mps;
    request.control_origin_speed_mps = origin.forward_velocity_mps;
    request.current_time_steering_rad = current.tire_steering_rad;
    request.current_steering_rad = origin.desired_steering_rad;
    request.current_response_steering_rad = origin.tire_steering_rad;
    request.current_lateral_velocity_mps = origin.lateral_velocity_mps;
    request.current_yaw_rate_radps = origin.yaw_rate_radps;
  };
  request.plan = materialized.plan;
  request.current_intent = materialized.plan->execution_artifact->identity.source_context.intent;
  request.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
  request.publication_prefix.reset();
  set_state(history->current, history->control_origin);
  const auto packet = retained::prospective_artifact_packet(request);
  if (!packet) throw std::runtime_error("materialized programme has no prospective packet");
  const auto prefix = vehicle::predict_prospective_publication(observation, *packet, model);
  if (!prefix) throw std::runtime_error("new current-world prefix unavailable");
  request.publication_prefix = prefix;
  set_state(prefix->current, prefix->control_origin);
  request.measured_to_control_path.clear(); request.measured_to_control_elapsed_sec.clear();
  for (const auto & sample : prefix->current_to_control) {
    request.measured_to_control_path.push_back({sample.state.x_m, sample.state.y_m, sample.state.yaw_rad});
    request.measured_to_control_elapsed_sec.push_back(sample.source_sec-request.now_sec);
  }
  YAML::Node output; output["authority"] = false;
  output["meaning"] = "Diagnostic current-world join of the previous actually published source's certified common Stop programme. Original observation/history retained; derived request explicitly rebound to that materialized programme, not relabeled as original captured request.";
  output["previous_decision"] = previous.decision_id;
  output["previous_source"] = previous.plan->execution_artifact->identity.sequence;
  output["original_program_rest_sec"] = accepted.proof->applied_program->tube().rest_sec;
  const auto remaining = m::mpcc_stop_input_program::remaining_program(
    accepted.proof->applied_program->prepared().program, request.now_sec);
  if (!remaining) throw std::runtime_error("original programme suffix unavailable");
  probe_program(output["publication_clock_suffix_only"], request, *remaining);
  auto held_first = original_request.publication_prefix->proposed_packet;
  auto held_brake = held_first;
  held_brake.published_sec += original_request.plan->execution_artifact->publication_interval_sec;
  held_brake.wire_acceleration_mps2 = static_cast<float>(original_request.minimum_acceleration_mps2);
  probe_program(output["diagnostic_same_first_packet_then_constant_steering_stop"], original_request,
    {original_request.plan->execution_artifact->publication_interval_sec, {held_first, held_brake}, true});
  audit(output["current_world_programme"], request);
  const auto joined = retained::evaluate(request);
  const auto production = m::mpcc_rate_resolved_production_adapter::build(joined);
  output["production"] = m::mpcc_rate_resolved_production_adapter::to_string(production.reason);
  YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter << output;
  std::ofstream(argv[2]) << emitter.c_str() << '\n';
}
