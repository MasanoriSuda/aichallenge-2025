#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_applied_program.hpp"
#include "multi_purpose_mpc_ros/detail/mpcc_footprint_enclosure.hpp"
#include "mpcc_rate_resolved_retained_revalidation.cpp"
#include <random>

namespace vehicle = m::mpcc_vehicle_model;
namespace num = vehicle::numerical;
namespace rec = m::recovery_footprint;

YAML::Node ranges(const num::Box & box)
{
  YAML::Node out(YAML::NodeType::Sequence);
  for (const auto & x : box) {
    YAML::Node row; row.push_back(x.lo); row.push_back(x.hi); out.push_back(row);
  }
  return out;
}

void audit(YAML::Node row, const retained::Request & request)
{
  row["decision"] = request.decision_id;
  row["source"] = request.plan->execution_artifact->identity.sequence;
  const auto actual = retained::evaluate(request);
  row["actual_reason"] = retained::to_string(actual.reason);
  row["actual_applied_reason"] = static_cast<int>(actual.applied_program_reason);
  row["actual_rejected_sec"] = actual.applied_program_rejected_sec;
  // Obtain a nominal trajectory as diagnostic input; it has no live authority.
  auto nominal_request = request;
  nominal_request.applied_program_required = false;
  nominal_request.input_application_profile.reset();
  // Keep the actual final reference arm. Merely disabling the applied gate
  // makes evaluate() stop at its first nominal success, potentially another arm.
  const auto profile = m::mpcc_rate_resolved_physical_adapter::build_terminal_stop_reference(
    *request.plan->execution_artifact, request.plan->physical_snapshot->terminal_stop_course_geometry);
  const auto nominal = actual.proof ? actual : retained::evaluate_with_stop_profile(
    nominal_request, actual.terminal_stop_reference_attempts > 1 || !profile ? nullptr : &*profile);
  row["nominal_reason"] = retained::to_string(nominal.reason);
  row["actual_reference_attempts"] = actual.terminal_stop_reference_attempts;
  if (!nominal.proof) return;
  const auto direct = m::mpcc_rate_resolved_applied_program::certify_terminal_stop(
    request, *nominal.proof, *request.input_application_profile);
  row["direct_applied_reason"] = static_cast<int>(direct.reason);
  row["direct_rejected_sec"] = direct.rejected_sec;
  if (direct.reason != actual.applied_program_reason ||
    (std::isfinite(direct.rejected_sec) && direct.rejected_sec != actual.applied_program_rejected_sec))
    throw std::runtime_error("diagnostic did not preserve the actual final reference arm");
  const auto & a = *request.plan->execution_artifact;
  const auto & model = a.vehicle_model;
  const auto & observation = request.publication_prefix->observation;
  const auto prepared = m::mpcc_stop_input_program::prepare({a.identity, request.decision_id,
    request.control_origin_sec, a.publication_interval_sec, request.publication_prefix->proposed_packet,
    model.steering_wire_gain, request.minimum_acceleration_mps2, request.maximum_acceleration_mps2,
    a.maximum_abs_steering_rad, a.maximum_abs_steering_rate_radps,
    a.physical_global_tolerance, nominal.proof->terminal_stop_actuation_samples});
  if (!prepared.prepared) throw std::runtime_error("original program unavailable");
  const auto & program = prepared.prepared->program;
  const auto prediction = vehicle::predict_applied_inputs_to_rest(
    observation, program, *request.input_application_profile, model);
  row["prediction_reason"] = static_cast<int>(prediction.reason);
  if (!prediction.tube) return;
  row["rest_sec"] = prediction.tube->rest_sec;
  row["now_sec"] = observation.now_sec;
  for (const auto & p : program.commands) {
    YAML::Node packet; packet.push_back(p.published_sec);
    packet.push_back(p.wire_acceleration_mps2); packet.push_back(p.wire_steering_rad);
    row["program"].push_back(packet);
  }
  const auto footprint = physical::resolve_clearance_footprint(
    request.current_footprint, request.plan->physical_snapshot->hard_wall_clearance_m);
  if (!footprint) throw std::runtime_error("original footprint unavailable");
  const auto to_world = [&](const rec::Pose2D & p) {
    const auto & o = observation.initial.state;
    const double c = std::cos(o.yaw_rad), s = std::sin(o.yaw_rad);
    return rec::Pose2D{o.x_m + c*p.x_m - s*p.y_m,
      o.y_m + s*p.x_m + c*p.y_m, o.yaw_rad + p.yaw_rad};
  };
  const auto wall_clear = [&](const num::Box & box) {
    const auto f = num::footprint(box, *footprint);
    const auto cells = rec::sample_footprint(*request.current_wall_grid, f.extents, to_world(f.pose));
    return cells.valid && !cells.out_of_map && cells.contact_cells.empty();
  };
  auto initial = observation.initial.state; initial.x_m = initial.y_m = initial.yaw_rad = 0;
  // Independent input channels: fixed corners, alternating extremes, random.
  // Finite scalar samples can falsify clearance, never certify the population.
  std::vector<vehicle::State> oracles(64, initial);
  std::mt19937_64 random(20260911); std::uniform_real_distribution<double> unit(0, 1);
  std::size_t step = 0, checks = 0, native_contacts = 0;
  double first_wall = NAN, first_native = NAN;
  for (const auto & sample : prediction.tube->source_to_rest) {
    num::Box enclosure;
    for (std::size_t i = 0; i < enclosure.size(); ++i)
      enclosure[i] = {sample.swept_body[i].lower, sample.swept_body[i].upper};
    std::vector<vehicle::ScalarRange> groups;
    for (const auto & group : sample.inputs.acceleration_sign_groups)
      if (group) groups.push_back(*group);
    std::optional<num::Box> native_hull;
    for (std::size_t j = 0; j < oracles.size(); ++j) {
      const auto & group = groups[j % groups.size()];
      const double af = j < 4 ? double(j % 2) : j < 8 ? double((step+j)%2) : unit(random);
      const double sf = j < 4 ? double(j / 2) : j < 8 ? double((step+j/2)%2) : unit(random);
      auto & state = oracles[j];
      state.desired_steering_rad = (sample.inputs.wire_steering_rad.lower + sf *
        (sample.inputs.wire_steering_rad.upper - sample.inputs.wire_steering_rad.lower)) / model.steering_wire_gain;
      const auto first = num::point(state);
      const auto next = vehicle::advance(state, {group.lower+af*(group.upper-group.lower), 0}, model, sample.duration_sec);
      if (!next) throw std::runtime_error("native oracle unavailable");
      state = next->state;
      const auto endpoint = num::point(state);
      for (std::size_t i = 0; i < endpoint.size(); ++i) {
        if (endpoint[i].lo < sample.endpoint_body[i].lower || endpoint[i].hi > sample.endpoint_body[i].upper)
          throw std::runtime_error("native state outside original enclosure");
        ++checks;
      }
      const auto segment = num::segment_box(first, endpoint);
      if (!native_hull) native_hull = segment;
      else for (std::size_t i = 0; i < segment.size(); ++i) (*native_hull)[i] = num::hull((*native_hull)[i], segment[i]);
      if (sample.begin_sec >= observation.now_sec && !wall_clear(segment)) {
        ++native_contacts;
        if (!std::isfinite(first_native)) first_native = sample.begin_sec;
      }
    }
    if (sample.begin_sec >= observation.now_sec && !std::isfinite(first_wall) && !wall_clear(enclosure)) {
      first_wall = sample.begin_sec;
      row["first_wall_box"] = ranges(enclosure);
      row["native_hull_at_first_wall"] = ranges(*native_hull);
      row["native_hull_clear_at_first_wall"] = wall_clear(*native_hull);
    }
    ++step;
  }
  row["first_wall_sec"] = first_wall;
  row["native_first_wall_sec"] = first_native;
  row["native_contact_segments"] = native_contacts;
  row["native_scalar_checks"] = checks;
  row["all_native_at_rest"] = std::all_of(oracles.begin(), oracles.end(), [](const auto & s) {
    return s.forward_velocity_mps == 0 && s.lateral_velocity_mps == 0 && s.yaw_rate_radps == 0;
  });
}

int main(int argc, char ** argv)
{
  if (argc != 3) return 2;
  const std::filesystem::path path = argv[1];
  const auto root = YAML::LoadFile(path.string());
  YAML::Node out; out["authority"] = false;
  out["meaning"] = "Original separate requests and actual final reference arms, no repaired source association. Full original numerical tube and sampled scalar input schedules. Native clearance does not prove every response clear.";
  for (const auto key : {"previous_accepted_revalidation_evidence", "revalidation_evidence"}) {
    const auto e = root[key]; if (!e || !e["request"]) continue;
    auto request = mpcc_observation::read_request(e["request"], path.parent_path());
    request.plan = read_plan(e["certified_plan_evidence"], path);
    out[key]["capture_status"] = e["status"];
    audit(out[key], request);
  }
  YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter << out;
  std::ofstream(argv[2]) << emitter.c_str() << '\n';
}
