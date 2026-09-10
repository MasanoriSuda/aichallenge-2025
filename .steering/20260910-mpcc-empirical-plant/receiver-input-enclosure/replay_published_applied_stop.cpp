#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_applied_program.hpp"
#include <chrono>

void report(YAML::Node row, const retained::Request & request)
{
  const auto started = std::chrono::steady_clock::now();
  const auto result = retained::evaluate(request);
  row["elapsed_ms"] = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  row["reason"] = retained::to_string(result.reason);
  const auto production = m::mpcc_rate_resolved_production_adapter::build(result);
  row["production"] = m::mpcc_rate_resolved_production_adapter::to_string(production.reason);
  row["applied_certificate"] = result.proof && bool(result.proof->applied_program);
  if (result.proof && result.proof->applied_program) {
    row["rest_sec"] = result.proof->applied_program->tube().rest_sec;
    row["minimum_peer_clearance_m"] = result.proof->applied_program->minimum_peer_clearance_m();
  }
  row["applied_reason"] = static_cast<int>(result.applied_program_reason);
  row["program_reason"] = static_cast<int>(result.applied_program_prepare_reason);
  row["prediction_reason"] = static_cast<int>(result.applied_input_prediction_reason);
  row["rejected_sec"] = result.applied_program_rejected_sec;
  // Obtain the old nominal proof only as diagnostic input. It cannot execute.
  auto nominal_request = request;
  nominal_request.applied_program_required = false;
  nominal_request.input_application_profile.reset();
  const auto nominal = retained::evaluate(nominal_request);
  row["nominal_only"] = retained::to_string(nominal.reason);
  if (!nominal.proof) return;
  const auto checked = m::mpcc_rate_resolved_applied_program::certify_terminal_stop(
    request, *nominal.proof, *request.input_application_profile);
  row["direct_applied_reason"] = static_cast<int>(checked.reason);
  const auto & artifact = *request.plan->execution_artifact;
  const auto prepared = m::mpcc_stop_input_program::prepare({artifact.identity, request.decision_id,
    request.control_origin_sec, artifact.publication_interval_sec,
    request.publication_prefix->proposed_packet, artifact.vehicle_model.steering_wire_gain,
    request.minimum_acceleration_mps2, request.maximum_acceleration_mps2,
    artifact.maximum_abs_steering_rad, artifact.maximum_abs_steering_rate_radps,
    artifact.physical_global_tolerance, nominal.proof->terminal_stop_actuation_samples});
  if (!prepared.prepared) return;
  const auto tube = m::mpcc_vehicle_model::predict_applied_inputs_to_rest(
    request.publication_prefix->observation, prepared.prepared->program,
    *request.input_application_profile, artifact.vehicle_model);
  row["direct_prediction"] = static_cast<int>(tube.reason);
  if (!tube.tube) return;
  row["rest_sec"] = tube.tube->rest_sec;
  row["publication_body"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto range : tube.tube->publication_body) {
    YAML::Node item; item.push_back(range.lower); item.push_back(range.upper);
    row["publication_body"].push_back(item);
  }
}

int main(int argc, char ** argv)
{
  if (argc != 3) return 2;
  const std::filesystem::path path = argv[1];
  const auto root = YAML::LoadFile(path.string());
  const auto e = root["revalidation_evidence"];
  auto request = mpcc_observation::read_request(e["request"], path.parent_path());
  request.plan = read_plan(e["certified_plan_evidence"], path);
  YAML::Node out;
  out["authority"] = false;
  out["source"] = request.plan->execution_artifact->identity.sequence;
  out["decision"] = request.decision_id;
  report(out["captured"], request);
  request.current_intent = request.plan->execution_artifact->identity.source_context.intent;
  report(out["published_upstream_retained"], request);
  const auto prepared = retained::evaluate_published_stop_successor(request);
  out["published_stop"] = retained::to_string(prepared.result.reason);
  if (prepared.request) {
    const auto candidate = bundle::build(*prepared.request, prepared.result, 100000U + request.decision_id);
    out["bundle"] = bundle::to_string(candidate.reason);
    if (candidate.plan) {
      auto joined = *prepared.request;
      joined.plan = candidate.plan;
      joined.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
      report(out["joined_published_stop"], joined);
    }
  }
  YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter << out;
  std::ofstream(argv[2]) << emitter.c_str() << '\n';
}
