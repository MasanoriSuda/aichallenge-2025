#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main

YAML::Node describe(const retained::Request & request)
{
  const auto result = retained::evaluate(request);
  YAML::Node n;
  n["decision"] = request.decision_id;
  n["source"] = request.plan->execution_artifact->identity.sequence;
  n["reason"] = retained::to_string(result.reason);
  n["continuation"] = m::mpcc_rate_resolved_physical_adapter::to_string(result.continuation_reason);
  n["lateral"] = result.current_control_state.lateral_m;
  n["speed"] = request.control_origin_speed_mps;
  n["initial_lower"] = request.plan->execution_artifact->lateral_lower_m.front();
  n["initial_upper"] = request.plan->execution_artifact->lateral_upper_m.front();
  n["terminal_certified"] = result.terminal_stop_certified;
  n["terminal_clearance"] = result.terminal_stop_minimum_dynamic_clearance_m;
  n["packet"] = m::mpcc_rate_resolved_production_adapter::to_string(
    m::mpcc_rate_resolved_production_adapter::build(result).reason);
  return n;
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
  out["authority"] = false; out["bound_comparison_solver_invocations"] = 0;
  out["exact_current"] = describe(request);
  // The first Cruise failure captured a still-Track source. Keep that exact
  // rejection and separately evaluate the same artifact under its own intent
  // to expose the physical boundary without claiming a real Track adoption.
  request.current_intent = request.plan->execution_artifact->identity.source_context.intent;
  out["counterfactual_source_intent"] = contract::to_string(request.current_intent);
  out["same_source_intent_original_bounds"] = describe(request);
  const auto & geometry = request.plan->physical_snapshot->terminal_stop_course_geometry;
  if (!m::mpcc_rate_resolved_physical_adapter::stop_course_geometry_valid(geometry) ||
    geometry.progress_m.front() != 0 || request.plan->execution_artifact->semantic_initial_state->progress_m != 0)
    throw std::runtime_error("diagnostic requires exact initial map knot");
  auto changed = *request.plan->execution_artifact;
  changed.lateral_lower_m.front() = geometry.lateral_lower_m.front();
  changed.lateral_upper_m.front() = geometry.lateral_upper_m.front();
  changed.identity.sequence += 100000;
  changed.identity.source_context.bounds_schema_id += "/diagnostic-initial-physical-support";
  changed.identity.source_context.fingerprint = contract::problem_context_fingerprint(changed.identity.source_context);
  auto p = *request.plan->physical_snapshot;
  p.identity.artifact = changed.identity;
  const auto native = m::mpcc_rate_resolved_physical_adapter::build(
    changed, changed.identity.source_context.intent, changed.identity.source_context.stage_geometry_id);
  if (!native.exact_trajectory) throw std::runtime_error("changed source native proof unavailable");
  p.trajectory = *native.exact_trajectory;
  const auto proof = physical::evaluate(p);
  const auto new_plan = certified::build(std::make_shared<const artifact::ExecutionArtifact>(changed), p, proof);
  if (!new_plan.plan) throw std::runtime_error("changed source physical certification unavailable");
  request.plan = new_plan.plan;
  out["initial_physical_support_only"] = describe(request);
  out["meaning"] = "Keep exact captured intent mismatch. Named comparison uses the source's original intent in both arms, then changes only initial lateral equality to sealed physical map support, resealing and recertifying the source; proposed packet/body/history/peers/future corridor unchanged. No command is sent.";
  std::string detail;
  const auto captured = snap::load_recorded_interaction_snapshot(path, &detail);
  if (!captured) throw std::runtime_error(detail);
  const auto inspect_producer = [&](const m::mpcc_rate_resolved_shadow::Snapshot & source, const char * key) {
    m::mpcc_rate_resolved_shadow::SolverContext solver;
    const auto rebuilt = solver.evaluate(source);
    auto producer = out[key];
    producer["meaning"] = "One SQP pipeline invocation on this exact captured source; no runtime authority";
    producer["outcome"] = m::mpcc_rate_resolved_shadow::to_string(rebuilt.outcome);
    producer["detail"] = rebuilt.detail;
    producer["source"] = source.identity.sequence;
    producer["physical_checked"] = rebuilt.post_refinement_physical_proof_checked;
    producer["physical_accepted"] = rebuilt.post_refinement_physical_proof_accepted;
    if (rebuilt.execution_artifact) {
      producer["initial_lower"] = rebuilt.execution_artifact->lateral_lower_m.front();
      producer["initial_upper"] = rebuilt.execution_artifact->lateral_upper_m.front();
      producer["semantic_initial_lateral"] = rebuilt.execution_artifact->semantic_initial_state->lateral_m;
      producer["qp_initial_lower"] = rebuilt.latest_state_feedback_preparation->final_problem.state_lower[0];
      producer["qp_initial_upper"] = rebuilt.latest_state_feedback_preparation->final_problem.state_upper[0];
    }
  };
  inspect_producer(captured->source, "current_snapshot_production_builder");
  YAML::Node envelope;
  envelope["source"] = e["inspected_source"];
  const auto inspected = snap::load_source_snapshot(envelope, path);
  if (!inspected) throw std::runtime_error("missing exact inspected source");
  inspect_producer(*inspected, "inspected_snapshot_production_builder");
  out["solver_pipeline_invocations"] = 2;
  YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter << out;
  std::ofstream(argv[2]) << emitter.c_str() << '\n';
  std::cout << emitter.c_str() << '\n';
  return 0;
}
