// Zero-solve regression on the sealed dev2-r8 failure. No runtime publisher.
#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main

namespace model = m::mpcc_vehicle_model;
namespace production = m::mpcc_rate_resolved_production_adapter;

int main(int argc, char ** argv)
{
  if (argc != 3) return 2;
  const std::filesystem::path path = argv[1];
  const auto root = YAML::LoadFile(path.string());
  const auto evidence = root["revalidation_evidence"];
  auto original = mpcc_observation::read_request(evidence["request"], path.parent_path());
  original.plan = read_plan(evidence["certified_plan_evidence"], path);
  const auto observation = snap::load_observation_provenance(
    root["source"]["semantic_request"]["observation_provenance"]);
  if (!observation) throw std::runtime_error("missing original observation");
  const auto & parameters = original.plan->execution_artifact->vehicle_model;
  YAML::Node out;
  out["authority"] = false;
  out["solver_invocations"] = 0;
  out["legacy_exact"] = retained::to_string(retained::evaluate(original).reason);
  const auto bind = [&](const model::PublishedCommand & packet) {
    const auto prediction = model::predict_prospective_publication(*observation, packet, parameters);
    if (!prediction) throw std::runtime_error("invalid proposed packet");
    auto r = original;
    const auto & state = prediction->control_origin;
    r.publication_prefix_required = true;
    r.publication_prefix = prediction;
    r.control_pose = {state.x_m, state.y_m, state.yaw_rad};
    r.current_speed_mps = prediction->current.forward_velocity_mps;
    r.control_origin_speed_mps = state.forward_velocity_mps;
    r.current_time_steering_rad = prediction->current.tire_steering_rad;
    r.current_steering_rad = state.desired_steering_rad;
    r.current_response_steering_rad = state.tire_steering_rad;
    r.current_lateral_velocity_mps = state.lateral_velocity_mps;
    r.current_yaw_rate_radps = state.yaw_rate_radps;
    r.measured_to_control_path.clear(); r.measured_to_control_elapsed_sec.clear();
    for (const auto & item : prediction->current_to_control) {
      r.measured_to_control_path.push_back({item.state.x_m, item.state.y_m, item.state.yaw_rad});
      r.measured_to_control_elapsed_sec.push_back(item.source_sec - r.now_sec);
    }
    return r;
  };
  const auto normal_packet = retained::prospective_artifact_packet(original);
  if (!normal_packet) throw std::runtime_error("normal artifact packet unavailable");
  const auto normal = retained::evaluate(bind(*normal_packet));
  out["bound_normal"] = retained::to_string(normal.reason);
  const model::PublishedCommand brake{
    original.now_sec, original.minimum_acceleration_mps2,
    static_cast<float>(original.previous_published_steering_rad * parameters.steering_wire_gain)};
  auto braking = bind(brake);
  const auto mismatch = retained::evaluate(braking);
  out["braking_prefix_normal_packet"] = retained::to_string(mismatch.reason);
  if (mismatch.reason != retained::Reason::PublicationPacketMismatch || production::build(mismatch).authority)
    throw std::runtime_error("accelerating packet borrowed braking prefix");
  for (const bool reproject : {false, true}) {
    auto r = braking;
    auto row = YAML::Node{};
    row["progress_projection"] = reproject ?
      "sensitivity: shift old progress by displacement in the first sealed course frame; actual runtime waypoint is not separately captured" :
      "original progress anchor retained; native continuation projects the changed body pose";
    if (reproject) {
      const auto heading = root["source"]["wall_course_frame_knots"][0]["heading_rad"].as<double>();
      r.control_origin_physical_progress_m +=
        std::cos(heading) * (r.control_pose.x_m - original.control_pose.x_m) +
        std::sin(heading) * (r.control_pose.y_m - original.control_pose.y_m);
    }
    const auto stop = retained::evaluate_stop_successor(r);
    row["stop"] = retained::to_string(stop.reason);
    row["control_speed"] = r.control_origin_speed_mps;
    row["clearance"] = stop.dynamic_clearance.minimum_clearance_m;
    const auto built = bundle::build(r, stop, 13000U + reproject);
    row["bundle"] = bundle::to_string(built.reason);
    if (!stop.accepted() || !built.plan) throw std::runtime_error("bound Stop failed");
    r.plan = built.plan;
    r.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
    const auto joined = retained::evaluate(r);
    const auto command = production::build(joined);
    row["join"] = retained::to_string(joined.reason);
    row["production"] = production::to_string(command.reason);
    if (!command.authority || !model::publication_packet_matches(*r.publication_prefix,
        r.now_sec, command.authority->command.acceleration_mps2,
        command.authority->command.steering_tire_angle_rad, parameters.steering_wire_gain))
      throw std::runtime_error("bound Stop packet mismatch");
    row["packet_acceleration"] = command.authority->command.acceleration_mps2;
    row["packet_steering"] = command.authority->command.steering_tire_angle_rad;
    const m::recovery_footprint::OccupancyGrid * inspected = nullptr, * observed = nullptr, * certified_grid = nullptr;
    auto encoded = snap::revalidation_evidence_node(&r, true, "roundtrip", inspected, observed, certified_grid);
    encoded["request"]["current_wall_grid"]["payload"] = evidence["request"]["current_wall_grid"]["payload"].as<std::string>();
    YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter << encoded;
    auto restored = mpcc_observation::read_request(YAML::Load(emitter.c_str())["request"], path.parent_path());
    restored.plan = built.plan;
    const auto replayed = retained::evaluate(restored);
    row["serialized_roundtrip"] = retained::to_string(replayed.reason);
    if (!production::build(replayed).authority || !restored.publication_prefix ||
        restored.publication_prefix->observation.commands.size() != observation->commands.size() ||
        restored.publication_prefix->observation.commands.back().published_sec != observation->commands.back().published_sec)
      throw std::runtime_error("roundtrip lost actual observation or valid binding");
    out["bound_braking"].push_back(row);
  }
  YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter << out;
  std::ofstream(argv[2]) << emitter.c_str() << '\n';
  std::cout << emitter.c_str() << '\n';
  return 0;
}
