#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main
#include <chrono>

// Reproduce the captured request without rebuilding it from later observations.
// Repeated calls measure warm/cold local work, not a live callback acceptance.
int main(int argc, char ** argv)
{
  if (argc != 3) return 2;
  const std::filesystem::path path = argv[1];
  const auto root = YAML::LoadFile(path.string());
  const auto evidence = root["revalidation_evidence"];
  auto request = mpcc_observation::read_request(evidence["request"], path.parent_path());
  request.plan = read_plan(evidence["certified_plan_evidence"], path);
  YAML::Node output;
  output["authority"] = false;
  output["capture_status"] = evidence["status"];
  output["recorded_detail"] = root["failure_detail"];
  output["decision"] = request.decision_id;
  output["source"] = request.plan->execution_artifact->identity.sequence;
  for (int attempt = 0; attempt < 5; ++attempt) {
    const auto started = std::chrono::steady_clock::now();
    const auto result = retained::evaluate(request);
    const double elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    YAML::Node row;
    row["elapsed_ms"] = elapsed;
    row["reason"] = retained::to_string(result.reason);
    row["proof_present"] = result.proof.has_value();
    row["feedback"] = result.feedback_shadow_attempted;
    row["feedback_proof_reason"] = retained::to_string(result.feedback_shadow_proof_reason);
    row["applied_reason"] = static_cast<int>(result.applied_program_reason);
    row["prediction_reason"] = static_cast<int>(result.applied_input_prediction_reason);
    row["rejected_sec"] = result.applied_program_rejected_sec;
    row["terminal_attempted"] = result.terminal_stop_attempted;
    row["terminal_certified"] = result.terminal_stop_certified;
    row["terminal_reason"] = static_cast<int>(result.terminal_stop_reason);
    row["terminal_exact_reason"] = static_cast<int>(result.terminal_stop_exact_reason);
    row["terminal_reference_attempts"] = result.terminal_stop_reference_attempts;
    const auto & r = result.runtime;
    auto runtime = row["runtime_ms"];
    runtime["pre"] = r.pre_continuation_ms;
    runtime["continuation_build"] = r.continuation_build_ms;
    runtime["continuation_proof"] = r.continuation_proof_ms;
    runtime["delay_wall"] = r.continuation_delay_wall_ms;
    runtime["dynamic"] = r.continuation_dynamic_ms;
    runtime["wall"] = r.continuation_wall_ms;
    runtime["terminal_build"] = r.terminal_build_ms;
    runtime["terminal_dynamic"] = r.terminal_dynamic_ms;
    runtime["terminal_wall"] = r.terminal_wall_ms;
    runtime["applied"] = r.applied_program_ms;
    output["attempts"].push_back(row);
  }
  YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter << output;
  std::ofstream(argv[2]) << emitter.c_str() << '\n';
}
