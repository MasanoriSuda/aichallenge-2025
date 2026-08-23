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
