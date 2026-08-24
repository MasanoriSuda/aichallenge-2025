"""Source-level deletion gates for the single-authority MPCC migration."""

from pathlib import Path
import re


SOURCE = (
    Path(__file__).resolve().parents[1] / "src" / "mpc_controller_cpp.cpp"
).read_text(encoding="utf-8")


def test_low_speed_direct_cannot_be_activated_as_production_authority() -> None:
    """The retired Gate2 bypass must not regain normal command ownership."""

    assert not re.search(r"low_speed_shift_control_active_\s*=\s*true", SOURCE)
    assert "return low_speed_shift_control(" not in SOURCE


def test_low_speed_direct_implementation_has_no_call_site() -> None:
    """Keep compatibility code unreachable until its final Slice 6 deletion."""

    assert len(re.findall(r"(?<![A-Za-z0-9_])low_speed_shift_control\(", SOURCE)) == 1


def test_overtake_worker_fresh_chain_uses_sealed_snapshot_context() -> None:
    """A tactical clone must not re-derive authority omitted from its snapshot."""

    worker_start = SOURCE.index("evaluate_overtake_canonical_worker_fresh(")
    worker_end = SOURCE.index(
        "void evaluate_overtake_canonical_retained_shadow(", worker_start
    )
    worker = SOURCE[worker_start:worker_end]

    assert "make_overtake_canonical_shadow_result(problem, snapshot_context)" in worker
    assert (
        "evaluate_overtake_canonical_fresh_shadow(\n"
        "      problem, extended_problem.value(), outcome.result.value(), now_sec,\n"
        "      execution_context)"
    ) in worker
    assert "make_problem_context(\n      problem" not in worker
    assert (
        "const auto execution_context = seal_problem_context_for_problem(\n"
        "      problem, snapshot_context, extended_problem->N)"
    ) in worker

    fresh_start = SOURCE.index(
        "OvertakeCanonicalFreshShadowResult\n"
        "  evaluate_overtake_canonical_fresh_shadow("
    )
    fresh_end = SOURCE.index(
        "evaluate_overtake_canonical_worker_fresh(", fresh_start
    )
    fresh = SOURCE[fresh_start:fresh_end]

    assert "const auto context = make_problem_context(" not in fresh
    assert "const auto & context = snapshot_context;" in fresh


def test_follow_qp_keeps_planning_and_physical_gap_contracts_separate() -> None:
    """Nominal feasibility must not consume the physical hard-gap boundary."""

    assert (
        "legacy.follow_longitudinal_contract.planning_gap_m" in SOURCE
    )
    assert (
        "legacy.follow_longitudinal_contract.target_progress_m[\n"
        "          static_cast<std::size_t>(state)] -\n"
        "          legacy.follow_longitudinal_contract.planning_gap_m"
    ) in SOURCE
    assert (
        "problem.follow_longitudinal_contract.hard_gap_m" in SOURCE
    )


def test_overtake_wall_proof_uses_exact_five_state_trajectory() -> None:
    """Wall proof must retain lag, heading and solved progress until certified."""

    branch_start = SOURCE.index("evaluate_extended_mpcc_branch(")
    branch_end = SOURCE.index(
        "evaluate_isolated_extended_mpcc_branch(", branch_start
    )
    branch = SOURCE[branch_start:branch_end]

    proof_input = branch.index("build_exact_extended_wall_proof_input(")
    physical_proof = branch.index("solved_mpcc_execution_path_wall_safe(")
    assert proof_input < physical_proof
    assert "exact_wall_proof->aligned" in branch
    assert "physical_execution_certificate_exact_trajectory" in branch
    assert "convert_extended_solution_to_legacy(" not in branch[:physical_proof]

    helper_start = SOURCE.index("build_exact_extended_wall_proof_input(")
    helper_end = SOURCE.index(
        "executed_extended_progress_solution_wall_safe(", helper_start
    )
    helper = SOURCE[helper_start:helper_end]
    assert "extract_extended_execution_trajectory(" in helper
    assert "exact.lag_m = extracted->lag_m" in helper
    assert "exact.heading_offset_rad = extracted->heading_offset_rad" in helper
    assert "exact.progress_m = extracted->progress_m" in helper


def test_overtake_entry_speed_proof_uses_selected_exact_trajectory() -> None:
    """Final prefix admission must not fall back to legacy rollout speed."""

    admission_start = SOURCE.index(
        "const auto certified_execution_minimum_speed = []"
    )
    admission_end = SOURCE.index(
        "if (async_shadow_enabled)", admission_start
    )
    admission = SOURCE[admission_start:admission_end]

    assert "mission.physical_execution_certificate_valid" in admission
    assert "exact_physical_execution_trajectory_complete(exact)" in admission
    assert "exact.velocity_mps.begin(), exact.velocity_mps.end()" in admission
    assert (
        "request.certified_execution_minimum_speed_mps =\n"
        "          certified_execution_minimum_speed(mission);"
    ) in admission


def test_bounded_overtake_prefix_has_one_horizon_owner_end_to_end() -> None:
    """Every pre-entry consumer must use the horizon built into the QP."""

    branch_start = SOURCE.index("evaluate_extended_mpcc_branch(")
    branch_end = SOURCE.index(
        "evaluate_isolated_extended_mpcc_branch(", branch_start
    )
    branch = SOURCE[branch_start:branch_end]
    built = branch.index("const auto extended = build_extended_progress_problem(")
    consumers = branch[built:]

    assert "const int effective_horizon = extended->N;" in consumers
    assert (
        "active_extended_branch_horizon_size_ = effective_horizon;" in consumers
    )
    assert "outcome.result->constraint_tolerance, effective_horizon" in consumers
    assert "const int nx_N = nx * (effective_horizon + 1);" in consumers
    assert "tracking_wp_id, effective_horizon," in consumers
    assert "for (int stage = 1; stage < effective_horizon + 1; ++stage)" in consumers
    assert "const int terminal = nx * effective_horizon;" in consumers
    assert (
        "legacy, std::move(prospective_context), effective_horizon)" in consumers
    )

    helper_start = SOURCE.index("build_exact_extended_wall_proof_input(")
    helper_end = SOURCE.index(
        "executed_extended_progress_solution_wall_safe(", helper_start
    )
    helper = SOURCE[helper_start:helper_end]
    assert "const int effective_horizon = extended_problem.N;" in helper
    assert "extract_extended_execution_trajectory(\n      primal, effective_horizon" in helper
    assert "for (int stage = 0; stage < effective_horizon; ++stage)" in helper

    production_start = SOURCE.index(
        "if (\n        problem.progress_contouring_active &&\n"
        "        cfg.progress_contouring.extended_dynamics_enabled)"
    )
    production_end = SOURCE.index(
        "record_solved_mpcc_execution_trajectory(", production_start
    )
    production = SOURCE[production_start:production_end]

    assert "executed_extended_progress_solution_wall_safe(" in production
    assert production.index(
        "executed_extended_progress_solution_wall_safe("
    ) < production.index("convert_extended_solution_to_legacy(")

    entry_start = SOURCE.index(
        "bool revalidate_overtake_entry_execution_certificate("
    )
    entry_end = SOURCE.index(
        "bool dynamic_margin_escape_solution_wall_safe(", entry_start
    )
    entry = SOURCE[entry_start:entry_end]
    assert "exact_physical_execution_trajectory_complete(" in entry
    assert "exact.lag_m" in entry
    assert "exact.heading_offset_rad" in entry
    assert "exact.progress_m" in entry
    assert "exact five-state certificate accepted" in entry
    assert "physical_execution_certificate_source_sec = now_sec" not in entry
    assert (
        "physical_execution_certificate_source_course_progress_m = model->s"
        not in entry
    )


def test_canonical_overtake_uses_production_wall_clearance_contract() -> None:
    """Fresh and retained candidates must not certify a zero-margin shadow."""

    fresh_start = SOURCE.index(
        "OvertakeCanonicalFreshShadowResult\n"
        "  evaluate_overtake_canonical_fresh_shadow("
    )
    fresh_end = SOURCE.index(
        "evaluate_overtake_canonical_worker_fresh(", fresh_start
    )
    fresh = SOURCE[fresh_start:fresh_end]
    physical_proof = fresh.index("solved_mpcc_execution_path_wall_safe(")
    physical_proof_end = fresh.index(
        "SolvedExecutionWallValidationScope::SweptFromCurrentPose",
        physical_proof,
    )
    proof_call = fresh[physical_proof:physical_proof_end]
    assert "problem.progress_execution_required_wall_clearance_m" in proof_call
    assert "upper_bound, 0.0" not in proof_call

    retained_start = SOURCE.index(
        "void evaluate_overtake_canonical_retained_shadow("
    )
    retained_end = SOURCE.index(
        "void invalidate_overtake_canonical_async_context()", retained_start
    )
    retained = SOURCE[retained_start:retained_end]
    assert (
        "proof_request.required_wall_clearance_m =\n"
        "      problem.progress_execution_required_wall_clearance_m"
    ) in retained


def test_overtake_intents_return_through_one_canonical_production_boundary() -> None:
    """ShiftOut/Pass/Return cannot fall through to an older normal solver."""

    resolver_start = SOURCE.index("canonical_overtake_production_control(")
    resolver_end = SOURCE.index("MpcControlCycleResult get_control(", resolver_start)
    resolver = SOURCE[resolver_start:resolver_end]

    assert "evaluate_overtake_async_shadow(problem, now_sec)" in resolver
    assert "return canonical_normal_control(" in resolver
    assert "return canonical_normal_emergency_stop(" in resolver
    assert "build_extended_progress_problem(" not in resolver
    assert "solve_extended_progress_problem(" not in resolver
    assert "evaluate_overtake_canonical_fresh_shadow(" not in resolver
    assert "convert_extended_solution_to_legacy(" not in resolver
    assert "solve_problem(" not in resolver
    assert "extended_progress_circuit_breaker_" not in resolver
    assert "extended_progress_reentry_gate_" not in resolver

    control_start = SOURCE.index("MpcControlCycleResult get_control(")
    old_path_start = SOURCE.index("Eigen::VectorXd dec;", control_start)
    canonical_boundary = SOURCE.index(
        "if (overtake_canonical_async_intent(control_intent))", control_start
    )
    assert canonical_boundary < old_path_start
    before_old_path = SOURCE[canonical_boundary:old_path_start]
    assert (
        "return canonical_overtake_production_control(\n"
        "          problem, now_sec, control_intent);"
    ) in before_old_path


def test_follow_async_snapshot_is_sealed_after_current_output_commit() -> None:
    """The next worker must inherit the steering published by this cycle."""

    control_start = SOURCE.index("MpcControlCycleResult get_control(")
    follow_start = SOURCE.index(
        "if (control_intent == mpcc_contract::ControlIntent::Follow)",
        control_start,
    )
    follow_end = SOURCE.index(
        "if (overtake_canonical_async_intent(control_intent))", follow_start
    )
    follow = SOURCE[follow_start:follow_end]

    consume = follow.index("evaluate_follow_async_shadow(problem, now_sec)")
    publish = follow.index("output = canonical_normal_control(")
    emergency = follow.index("output = canonical_normal_emergency_stop(")
    submit = follow.index("submit_follow_canonical_async(problem, now_sec)")
    assert consume < publish < submit
    assert consume < emergency < submit

    evaluator_start = SOURCE.index("FollowShadowCycleResult evaluate_follow_async_shadow(")
    evaluator_end = SOURCE.index(
        "void record_follow_canonical_async_status", evaluator_start
    )
    evaluator = SOURCE[evaluator_start:evaluator_end]
    assert "submit_follow_canonical_async(problem, now_sec)" not in evaluator


def test_follow_transition_admission_uses_the_same_canonical_producer() -> None:
    """Intent elevation must be atomic with a current executable Follow plan."""

    admission_start = SOURCE.index(
        "FollowShadowCycleResult evaluate_follow_transition_admission("
    )
    admission_end = SOURCE.index(
        "void record_follow_canonical_async_status", admission_start
    )
    admission = SOURCE[admission_start:admission_end]

    assert "evaluate_follow_fresh_shadow(problem, now_sec, context)" in admission
    assert (
        "evaluate_follow_retained_shadow(problem, now_sec, fresh_plan, result)"
        in admission
    )
    assert "follow_canonical_lifecycle_->plan_store.replace(*fresh_plan)" in admission
    assert "solve_problem(" not in admission
    assert "legacy-mpc" not in admission
    assert "safe_failure_control(" not in admission

    control_start = SOURCE.index("MpcControlCycleResult get_control(")
    follow_start = SOURCE.index(
        "if (control_intent == mpcc_contract::ControlIntent::Follow)",
        control_start,
    )
    follow_end = SOURCE.index(
        "if (overtake_canonical_async_intent(control_intent))", follow_start
    )
    follow = SOURCE[follow_start:follow_end]
    assert "SolveTransitionAdmission" in follow
    assert "evaluate_follow_transition_admission(problem, now_sec)" in follow
    assert "last_published_canonical_intent_" in follow


def test_follow_fresh_solver_transaction_is_serialized_end_to_end() -> None:
    """Worker and transition admission cannot interleave solve/warm publication."""

    lifecycle_start = SOURCE.index("struct CanonicalNormalLifecycle")
    lifecycle_end = SOURCE.index("struct MPC", lifecycle_start)
    lifecycle = SOURCE[lifecycle_start:lifecycle_end]
    assert "std::mutex solver_transaction_mutex" in lifecycle

    fresh_start = SOURCE.index("FollowShadowCycleResult evaluate_follow_fresh_shadow(")
    fresh_end = SOURCE.index("void invalidate_follow_canonical_async_context()", fresh_start)
    fresh = SOURCE[fresh_start:fresh_end]
    transaction = fresh.index("solver_transaction_mutex")
    solve = fresh.index("solve_extended_progress_problem(")
    publish_warm = fresh.index("publish_certified_extended_progress_warm_start(")
    assert transaction < solve < publish_warm


def test_overtake_entry_adopts_the_already_solved_canonical_artifact() -> None:
    """ShiftOut may not commit first and launch a duplicate initial solve later."""

    branch_start = SOURCE.index("evaluate_extended_mpcc_branch(")
    branch_end = SOURCE.index(
        "evaluate_isolated_extended_mpcc_branch(", branch_start
    )
    branch = SOURCE[branch_start:branch_end]
    solve = branch.index("solve_extended_progress_problem(")
    target_proof = branch.index("build_current_overtake_target_tube(")
    canonical = branch.index("evaluate_overtake_canonical_fresh_shadow(")
    assert solve < target_proof < canonical
    assert "legacy.progress_execution_target_exclusion_certified = true" in branch
    assert "preentry_canonical_plan" in branch

    entry_start = SOURCE.index("const bool fresh_normal_mission_entry =")
    entry_end = SOURCE.index(
        "transition_overtake_line_phase(\n"
        "          direct_pass ? OvertakeLinePhase::Pass",
        entry_start,
    )
    entry = SOURCE[entry_start:entry_end]
    assert "resolve_overtake_preentry_plan(" in entry
    assert "prepare_overtake_canonical_async_context(" in entry
    assert "plan_store.replace(" in entry
    assert entry.index("freeze_selected_overtake_mission(") < entry.index(
        "prepare_overtake_canonical_async_context("
    )

    certificate_start = SOURCE.index(
        "bool revalidate_overtake_entry_execution_certificate("
    )
    certificate_end = SOURCE.index(
        "bool dynamic_margin_escape_solution_wall_safe(", certificate_start
    )
    certificate = SOURCE[certificate_start:certificate_end]
    assert "const canonical_plan::CanonicalExecutionPlan & preentry_plan" in certificate
    assert "sample_retained_progress_advance(" in certificate
    assert "preentry_plan.predicted_states" in certificate
    assert "&plan_time_sec" in certificate
    assert "build_current_overtake_target_tube(" in certificate
    assert "validate_frenet_dp_target_bound_horizon(" in certificate

    retained_start = SOURCE.index(
        "void evaluate_overtake_canonical_retained_shadow("
    )
    retained_end = SOURCE.index(
        "void invalidate_overtake_canonical_async_context()", retained_start
    )
    retained = SOURCE[retained_start:retained_end]
    assert "sample_retained_progress_advance(" in retained
    assert "build_retained_lateral_corridor(" in retained
    assert (
        "corridor.lateral_lower_m = retained_lateral_corridor->lower_m"
        in retained
    )
    assert (
        "corridor.lateral_upper_m = retained_lateral_corridor->upper_m"
        in retained
    )
    assert "problem.progress_state_lower[" not in retained
    assert "problem.progress_state_upper[" not in retained
    assert "&corridor.elapsed_time_sec" in retained
    assert (
        "corridor_path_distance_m.push_back(\n"
        "        problem.progress_stage_geometry"
    ) not in retained
    assert "intersect_overtake_target_tube(" in retained
    assert "stage_corridor_mpc_target_bound_was_active_ ||" not in retained

    fresh_start = SOURCE.index(
        "OvertakeCanonicalFreshShadowResult\n"
        "  evaluate_overtake_canonical_fresh_shadow("
    )
    fresh_end = SOURCE.index(
        "evaluate_overtake_canonical_worker_fresh(", fresh_start
    )
    fresh = SOURCE[fresh_start:fresh_end]
    assert "extraction.lateral_lower_m.push_back(" in fresh
    assert "extraction.lateral_upper_m.push_back(" in fresh


def test_runtime_overtake_replacement_is_a_typed_canonical_artifact() -> None:
    """Runtime Mission replacement may not drop the selected five-state plan."""

    artifact_start = SOURCE.index("struct OvertakeExecutionArtifact")
    artifact_end = SOURCE.index("struct V2XBehaviorOutput", artifact_start)
    artifact = SOURCE[artifact_start:artifact_end]
    assert "OvertakeMissionCandidate mission" in artifact
    assert "CanonicalExecutionPlan> canonical_plan" in artifact
    assert "make_overtake_execution_artifact(" in artifact

    behavior_start = SOURCE.index("struct V2XBehaviorOutput")
    behavior_end = SOURCE.index("struct ExtendedMpccBranchArtifact", behavior_start)
    behavior = SOURCE[behavior_start:behavior_end]
    assert "mpcc_lite_same_side_replan_artifact" in behavior
    assert "mpcc_lite_cross_side_replan_artifact" in behavior
    assert "mpcc_lite_same_side_replan_mission" not in behavior
    assert "mpcc_lite_cross_side_replan_mission" not in behavior

    branch_start = SOURCE.index("evaluate_extended_mpcc_branch(")
    branch_end = SOURCE.index(
        "evaluate_isolated_extended_mpcc_branch(", branch_start
    )
    branch = SOURCE[branch_start:branch_end]
    assert (
        "artifact_identity.source_mission_generation + 1U" in branch
    )
    assert "overtake_line_state_.mission_generation + 1U" not in branch

    selector_start = SOURCE.index(
        "void evaluate_and_select_extended_mpcc_branches("
    )
    selector_end = SOURCE.index(
        "bool submit_mpcc_lite_async_snapshot(", selector_start
    )
    selector = SOURCE[selector_start:selector_end]
    assert "const OvertakeArtifactIdentitySeed & artifact_identity" in selector
    assert "artifact_identity.source_side_sign" in selector
    assert "artifact_identity.source_phase" in selector

    worker_start = SOURCE.index("const auto submission = mpcc_lite_async_worker_->submit_latest(")
    worker_end = SOURCE.index("mpcc_lite_async_last_snapshot_ms_", worker_start)
    worker = SOURCE[worker_start:worker_end]
    assert "const OvertakeArtifactIdentitySeed artifact_identity{" in worker
    assert (
        "evaluate_and_select_extended_mpcc_branches(\n"
        "            result.behavior, horizon_size, now_sec, artifact_identity)"
        in worker
    )

    replace_start = SOURCE.index(
        "bool replace_frozen_overtake_mission_after_dynamic_replan("
    )
    replace_end = SOURCE.index("double mission_completion_reserve_sec() const", replace_start)
    replace = SOURCE[replace_start:replace_end]
    assert "replacement_canonical_plan" in replace
    assert "resolve_overtake_preentry_plan(" in replace
    assert "prospective_generation" in replace
    assert "adopt_overtake_canonical_plan_context(" in replace
    assert "prepare_overtake_canonical_async_context(" not in replace
    assert replace.index("freeze_selected_overtake_mission(") < replace.index(
        "adopt_overtake_canonical_plan_context("
    )
    assert replace.index("adopt_overtake_canonical_plan_context(") < replace.index(
        "transition_overtake_line_phase("
    )

    adoption_start = SOURCE.index(
        "OvertakeCanonicalPlanAdoption adopt_overtake_canonical_plan_context("
    )
    adoption_end = SOURCE.index(
        "bool submit_overtake_canonical_async(", adoption_start
    )
    adoption = SOURCE[adoption_start:adoption_end]
    assert adoption.index("plan_store.replace(") < adoption.index(
        "overtake_canonical_async_context_ = resolution.next"
    )
    assert "plan_store.clear(" not in adoption

    runtime_start = SOURCE.index(
        "behavior_output.mpcc_lite_same_side_replan_artifact.has_value()"
    )
    runtime_end = SOURCE.index(
        "behavior_output.opponent_side_replan_ready", runtime_start
    )
    runtime = SOURCE[runtime_start:runtime_end]
    assert "artifact.canonical_plan" in runtime
    assert "locked_target_observation_generation" in runtime


def test_overtake_preentry_target_prediction_is_an_explicit_snapshot_contract() -> None:
    """Idle-side workers cannot depend on a committed target that does not exist yet."""

    behavior_start = SOURCE.index("struct V2XBehaviorOutput")
    behavior_end = SOURCE.index("struct ExtendedMpccBranchArtifact", behavior_start)
    behavior = SOURCE[behavior_start:behavior_end]
    assert "target_execution_prediction_valid" in behavior
    assert "target_execution_predicted_longitudinal" in behavior
    assert "target_execution_predicted_lateral" in behavior

    selection_start = SOURCE.index("const auto set_target_execution_prediction =")
    selection_end = SOURCE.index(
        "output.overtake_entry_target_speed =", selection_start
    )
    selection = SOURCE[selection_start:selection_end]
    assert "nearest_front_course_lateral" in selection
    assert "nearest_front_lateral_velocity_valid" in selection
    assert "target_execution_prediction_valid" in selection

    tube_start = SOURCE.index("CurrentOvertakeTargetTube build_current_overtake_target_tube(")
    tube_end = SOURCE.index(
        "bool revalidate_overtake_entry_execution_certificate(", tube_start
    )
    tube = SOURCE[tube_start:tube_end]
    assert "committed_target_prediction_available" in tube
    assert "entry_target_prediction_available" in tube
    assert "behavior.target_execution_predicted_lateral" in tube
    assert "behavior.target_execution_predicted_longitudinal" in tube



def test_unresolved_dynamic_wait_cannot_fall_through_to_legacy_normal() -> None:
    """A broken ShiftOut/Pass wait handoff must fail closed, not change authority."""

    scope_start = SOURCE.index("unresolved_dynamic_wait_canonical_scope() const")
    scope_end = SOURCE.index(
        "void invalidate_overtake_canonical_async_context()", scope_start
    )
    scope = SOURCE[scope_start:scope_end]
    assert "authority.request.dynamic_wait_active" in scope
    assert (
        "authority.resolution.action == overtake_orchestrator::Action::DynamicWait"
        in scope
    )

    control_start = SOURCE.index("MpcControlCycleResult get_control(")
    old_path_start = SOURCE.index("Eigen::VectorXd dec;", control_start)
    fail_closed = SOURCE.index(
        "if (unresolved_dynamic_wait_canonical_scope())", control_start
    )
    assert fail_closed < old_path_start
    before_old_path = SOURCE[fail_closed:old_path_start]
    assert "return canonical_normal_emergency_stop(" in before_old_path
    assert "dynamic wait has no executable canonical lateral authority" in before_old_path


def test_rejoin_uses_isolated_canonical_production_without_legacy_fallthrough() -> None:
    """Qualified Rejoin must publish canonical or Emergency, never legacy normal."""

    activation_start = SOURCE.index(
        "const bool progress_contouring_execution_phase ="
    )
    activation_end = SOURCE.index(
        "const bool dynamic_escape_formulation_lease_active", activation_start
    )
    activation = SOURCE[activation_start:activation_end]
    assert "OvertakeLinePhase::Recovery" not in activation

    evaluator_start = SOURCE.index("evaluate_canonical_normal_shadow(")
    evaluator_end = SOURCE.index(
        "resolve_physically_validated_mpcc_execution_trajectory(", evaluator_start
    )
    evaluator = SOURCE[evaluator_start:evaluator_end]
    assert "rejoin_shadow_plan_store_" in evaluator
    assert "rejoin_shadow_warm_start_identity_" in evaluator
    assert "rejoin_shadow_solver_context_" in evaluator
    assert "if (!rejoin_mode)" in evaluator
    assert "Rejoin retained policy intentionally unavailable" in evaluator

    control_start = SOURCE.index("MpcControlCycleResult get_control(")
    old_path_start = SOURCE.index("Eigen::VectorXd dec;", control_start)
    rejoin_shadow = SOURCE.index(
        "if (problem.rejoin_shadow_requested)", control_start
    )
    assert rejoin_shadow < old_path_start
    observation = SOURCE[rejoin_shadow:old_path_start]
    assert "CanonicalNormalShadowMode::Rejoin" in observation
    assert "record_rejoin_canonical_telemetry" in observation
    assert "return canonical_normal_control(" in observation
    assert "return canonical_normal_emergency_stop(" in observation
    assert "canonical_result.selected.complete()" in observation
    assert "legacy command" not in observation

    telemetry_start = SOURCE.index("void record_rejoin_canonical_telemetry(")
    telemetry_end = SOURCE.index("void record_overtake_canonical", telemetry_start)
    telemetry = SOURCE[telemetry_start:telemetry_end]
    assert "production_authority=canonical" in telemetry
    assert "authority=production" in telemetry
    assert "legacy-unchanged" not in telemetry


def test_extended_first_stage_linearization_is_anchored_to_the_execution_state() -> None:
    """The first affine dynamics block must be tangent at the fixed state zero."""

    builder_start = SOURCE.index(
        "std::optional<ExtendedProgressMpcProblem> build_extended_progress_problem("
    )
    builder_end = SOURCE.index(
        "relinearize_extended_progress_wall_bounds(", builder_start
    )
    builder = SOURCE[builder_start:builder_end]
    assert "const bool initial_stage = stage == 0;" in builder
    assert "initial_stage ? initial_frenet_pose->lateral_m" in builder
    assert "initial_stage ? initial_lag_m" in builder
    assert re.search(
        r"initial_stage\s*\?\s*initial_frenet_pose->heading_offset_rad", builder
    )
    assert re.search(
        r"initial_stage\s*\?\s*legacy\.progress_measured_speed_mps", builder
    )
    assert "initial_stage ? previous_curvature" in builder
    assert "legacy.progress_stage_dt_sec[static_cast<std::size_t>(stage)]" in builder


def test_canonical_overtake_wall_certificate_is_not_reinterpreted_downstream() -> None:
    """The legacy x/y wall monitor cannot replace a certified canonical command."""

    monitor_start = SOURCE.index(
        "const bool active_overtake_wall_monitor_relevant ="
    )
    monitor_end = SOURCE.index(
        "const int wall_path_scan_interval_cycles", monitor_start
    )
    monitor = SOURCE[monitor_start:monitor_end]

    assert "!canonical_normal_command.has_value()" in monitor
    assert "!canonical_emergency_stop" in monitor

    ownership_start = SOURCE.index(
        "const auto legacy_wall_handoff_authority ="
    )
    ownership_end = SOURCE.index(
        "const auto retained_dynamic_escape_snapshot", ownership_start
    )
    ownership = SOURCE[ownership_start:ownership_end]
    assert "resolve_legacy_wall_handoff_authority(" in ownership
    retirement = "mpc_->retire_legacy_dynamic_escape_execution();"
    assert ownership.count(retirement) == 1
    assert ownership.index(retirement) < ownership.index(
        "const bool legacy_state_present ="
    )
    legacy_state_expression = ownership[
        ownership.index("const bool legacy_state_present =") :
        ownership.index("solver_wall_handoff_admission_gate_.reset()")
    ]
    assert retirement not in legacy_state_expression
    assert "solver_wall_handoff_admission_gate_.reset()" in ownership
    assert "overtake_wall_admission_gate_.reset()" in ownership
    assert "dynamic_escape_wall_admission_gate_.reset()" in ownership
    assert "dynamic_escape_exit_gate_.reset()" in ownership

    dynamic_monitor_start = SOURCE.index(
        "const bool dynamic_escape_wall_monitor_relevant ="
    )
    dynamic_monitor_end = SOURCE.index(
        "const bool dynamic_escape_wall_scan_due", dynamic_monitor_start
    )
    dynamic_monitor = SOURCE[dynamic_monitor_start:dynamic_monitor_end]
    assert (
        "legacy_wall_handoff_authority.legacy_normal_handoff_allowed"
        in dynamic_monitor
    )

    solver_recovery_start = SOURCE.index(
        "const bool recovered_from_bounded_continuation ="
    )
    solver_recovery_end = SOURCE.index(
        "if (!enable_control_", solver_recovery_start
    )
    solver_recovery = SOURCE[solver_recovery_start:solver_recovery_end]
    assert (
        "legacy_wall_handoff_authority.legacy_normal_handoff_allowed"
        in solver_recovery
    )


def test_overtake_physical_failure_preserves_stage_and_authority_provenance() -> None:
    """A hard physical rejection must identify both proof and plan producers."""

    horizon_start = SOURCE.index("struct OvertakeLineHorizonEvaluation")
    horizon_end = SOURCE.index(
        "enum class OvertakeRecedingHorizonFailureKind", horizon_start
    )
    horizon = SOURCE[horizon_start:horizon_end]
    assert "physical_failure_cause" in horizon
    assert "physical_failure_path_distance_m" in horizon
    assert "physical_failure_lateral_target_m" in horizon
    assert "physical_failure_wall_lower_m" in horizon
    assert "physical_failure_wall_upper_m" in horizon
    assert "physical_failure_required_lateral_accel_mps2" in horizon

    validation_start = SOURCE.index("const auto record_validation_failure =")
    validation_end = SOURCE.index(
        "if (!validation_accepted) {", validation_start
    )
    validation = SOURCE[validation_start:validation_end]
    assert "physical_validation_attempt_count" in validation
    assert "candidate_speed_mps" in validation
    assert "wall_clearance_m" in validation
    assert "initial_contract" in validation

    trace_start = SOURCE.index("OvertakeLine physical authority failure:")
    trace_end = SOURCE.index(
        "const bool rear_clear_return_handoff", trace_start
    )
    trace = SOURCE[trace_start:trace_end]
    for token in (
        "generation=%lu",
        "cause=%s",
        "stage=%d",
        "wall=[%.3f,%.3f]",
        "initial_contract=%d",
        "attempts=%zu",
        "dp=%d/authority=%d/runtime=%d/side=%d/source_age=%.3f",
        "canonical=%d/plan=%lu/generation=%lu/side=%d/fingerprint=%lu/",
    ):
        assert token in trace


def test_canonical_overtake_demotes_legacy_mission_viability_owner() -> None:
    """A complete reference may feed canonical MPCC but cannot end its Mission."""

    resolution_start = SOURCE.index(
        "const auto receding_viability_authority ="
    )
    transition_start = SOURCE.index(
        "if (!horizon_evaluation.execution_feasible())", resolution_start
    )
    transition_end = SOURCE.index(
        "double previous_target_ey = current_ey;", transition_start
    )
    transition = SOURCE[transition_start:transition_end]

    assert "receding_viability_reference_only" in transition
    assert "OvertakeLine legacy viability demoted:" in transition
    demotion_start = transition.index("if (receding_viability_reference_only)")
    legacy_start = transition.index(
        "should_terminate_recovery_retained_mission", demotion_start
    )
    demotion = transition[demotion_start:legacy_start]
    assert "transition_overtake_line_phase" not in demotion
    assert "enter_dynamic_mission_wait" not in demotion
    assert "arm_overtake_line_side_retry_block" not in demotion
    assert "reset_overtake_line_state" not in demotion

    speed_cap_start = SOURCE.index(
        "horizon_evaluation.static_map_wall_infeasible", resolution_start
    )
    speed_cap_end = SOURCE.index(
        "if (!horizon_evaluation.execution_feasible())", speed_cap_start
    )
    assert "!receding_viability_reference_only" in SOURCE[
        speed_cap_start:speed_cap_end
    ]

    trace_start = SOURCE.index("OvertakeLine physical authority failure:")
    trace_end = SOURCE.index(
        "const bool rear_clear_return_handoff", trace_start
    )
    trace = SOURCE[trace_start:trace_end]
    assert "viability=%s/%s/reference_complete=%d" in trace


def test_runtime_wall_preplanner_cannot_destroy_canonical_mission() -> None:
    """An optional wall-reference producer has no FSM or Recovery authority."""

    assert "RuntimeWallPreplanAction::ExitCurrentMission" not in SOURCE
    prefix_start = SOURCE.index(
        "OvertakeLine runtime wall escape prefix commit rejected:"
    )
    prefix_end = SOURCE.index(
        "const auto transition_action =", prefix_start
    )
    prefix_failure = SOURCE[prefix_start:prefix_end]

    assert "enter_dynamic_mission_wait" not in prefix_failure
    assert "transition_overtake_line_phase" not in prefix_failure
    assert "reset_overtake_line_state" not in prefix_failure
    assert "canonical-reference-only" in prefix_failure


def test_rate_resolved_track_cruise_runtime_is_observation_only() -> None:
    """The new six-state runtime path may solve and log, but never own control."""

    submit_start = SOURCE.index(
        "bool submit_rate_resolved_track_cruise_shadow("
    )
    evaluator_start = SOURCE.index(
        "TrackCruiseShadowCycleResult evaluate_canonical_normal_shadow("
    )
    transport = SOURCE[submit_start:evaluator_start]
    assert "rate_resolved_track_cruise_shadow_worker_->submit_latest(" in transport
    assert "rate_resolved_track_cruise_shadow_mailbox_->latest_after(" in transport
    assert "authority=shadow, selected=0" in transport
    for forbidden in (
        "canonical_normal_control(",
        "canonical_normal_emergency_stop(",
        "track_cruise_shadow_plan_store_.publish(",
        "record_solution_contract(",
        "previous_steering =",
        "current_control =",
    ):
        assert forbidden not in transport

    branch_start = SOURCE.index("if (problem.track_cruise_shadow_requested)")
    branch_end = SOURCE.index("if (problem.rejoin_shadow_requested)", branch_start)
    branch = SOURCE[branch_start:branch_end]
    assert "record_rate_resolved_track_cruise_shadow(problem, now_sec);" in branch
    assert "canonical_result.selected.complete()" in branch
    assert "rate_resolved" not in branch[branch.index("if (canonical_result") :]


def test_rate_resolved_shadow_replaces_legacy_first_curvature_time_base() -> None:
    """Six-state reachability must not inherit the old one-cycle curvature patch."""

    builder_start = SOURCE.index(
        "std::optional<ExtendedProgressMpcProblem> build_extended_progress_problem("
    )
    builder_end = SOURCE.index(
        "relinearize_extended_progress_wall_bounds(", builder_start
    )
    builder = SOURCE[builder_start:builder_end]
    semantic_start = builder.index("rate_resolved_shadow_request.emplace();")
    semantic_end = builder.index("Eigen::VectorXd q =", semantic_start)
    semantic = builder[semantic_start:semantic_end]
    assert "request.current_steering_rad = previous_steering;" in semantic
    assert "request.maximum_abs_steering_rate_radps" in semantic
    assert "first_curvature_input_lower" in semantic
    assert "first_curvature_input_upper" in semantic
    assert "physical_first_curvature->reachable_lower_radpm" not in semantic
    assert "physical_first_curvature->reachable_upper_radpm" not in semantic
