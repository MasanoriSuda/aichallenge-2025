#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_applied_program.hpp"
#include <chrono>

// Diagnostic replay: original requests are never rebound or repaired. The
// additional profile is an explicitly named counterfactual for old snapshots.
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
    const auto evidence = root[key];
    auto row = out[key];
    if (!evidence || !evidence["request"] || !evidence["certified_plan_evidence"]) {
      row["status"] = "unavailable";
      continue;
    }
    row["capture_status"] = evidence["status"].as<std::string>();
    row["capture_reason"] = evidence["reason"].as<std::string>();
    auto request = mpcc_observation::read_request(evidence["request"], path.parent_path());
    request.plan = read_plan(evidence["certified_plan_evidence"], path, evidence["inspected_source"]);
    row["decision"] = request.decision_id;
    row["source"] = request.plan->execution_artifact->identity.sequence;
    for (const bool profiled : {false, true}) {
      auto r = request;
      auto result_row = row[profiled ? "with_empirical_profile" : "captured_request"];
      if (profiled) {
        r.applied_program_required = true;
        r.input_application_profile = {"awsim-2025-empirical-receiver-age-250ms-v1", .25, .25, .1};
      }
      const auto started = std::chrono::steady_clock::now();
      const auto result = retained::evaluate(r);
      result_row["elapsed_ms"] = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      result_row["reason"] = retained::to_string(result.reason);
      result_row["source_horizon_program"] = result.terminal_stop_source_horizon_program;
      result_row["terminal_attempts"] = result.terminal_stop_reference_attempts;
      if(result.terminal_stop_forward_velocity_ceiling_mps)
        result_row["forward_velocity_ceiling_mps"] = *result.terminal_stop_forward_velocity_ceiling_mps;
      result_row["applied_reason"] = static_cast<int>(result.applied_program_reason);
      result_row["program_reason"] = static_cast<int>(result.applied_program_prepare_reason);
      result_row["prediction_reason"] = static_cast<int>(result.applied_input_prediction_reason);
      result_row["rejected_sec"] = result.applied_program_rejected_sec;
      result_row["rejected_peer"] = result.applied_program_rejected_peer_id;
      const auto publication = m::mpcc_rate_resolved_production_adapter::build(result);
      result_row["production"] = m::mpcc_rate_resolved_production_adapter::to_string(publication.reason);
      if (result.reason == retained::Reason::Accepted && !publication.authority)
        throw std::runtime_error("accepted proof lost production identity");
      if (!result.proof || !result.proof->applied_program) continue;
      const auto & applied = *result.proof->applied_program;
      if (!applied.matches(r) || !applied.matches(*result.proof))
        throw std::runtime_error("certificate mismatch");
      result_row["rest_sec"] = applied.tube().rest_sec;
      result_row["program_commands"] = applied.prepared().program.commands.size();
      result_row["samples"] = applied.checked_samples();
      result_row["peer_clearance_m"] = applied.minimum_peer_clearance_m();
      const auto stop = bundle::build_certified_terminal(r, result, 100000U + r.decision_id);
      result_row["materialization"] = bundle::to_string(stop.reason);
      if (stop.plan) {
        r.plan = stop.plan;
        r.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
        const auto joined = retained::evaluate(r);
        result_row["join"] = retained::to_string(joined.reason);
        result_row["join_applied_reason"] = static_cast<int>(joined.applied_program_reason);
      }
    }
  }
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(17);
  emitter << out;
  std::ofstream(argv[2]) << emitter.c_str() << '\n';
  return 0;
}
