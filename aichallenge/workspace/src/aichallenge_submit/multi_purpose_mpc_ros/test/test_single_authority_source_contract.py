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
