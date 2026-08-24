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
        "      snapshot_context)"
    ) in worker
    assert "make_problem_context(\n      problem" not in worker
    assert (
        "seal_problem_context_for_problem(\n"
        "      problem, snapshot_context)"
    ) in worker

    fresh_start = SOURCE.index("evaluate_overtake_canonical_fresh_shadow(")
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

    fresh_start = SOURCE.index("evaluate_overtake_canonical_fresh_shadow(")
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


def test_rejoin_is_observed_by_an_isolated_shadow_without_production_promotion() -> None:
    """Recovery evidence must not silently promote or share Track/Cruise state."""

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
    assert "record_rejoin_shadow_telemetry" in observation
    assert "return canonical_normal_control(" not in observation
    assert "return canonical_normal_emergency_stop(" not in observation


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
