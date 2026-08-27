"""Evidence-only A--D architecture comparison classification."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


_SOLVE_STATES = {"succeeded", "failed", "not_run"}
_PROOF_STATES = {"accepted", "failed", "not_run"}


@dataclass(frozen=True)
class Classification:
    classification: str
    reason: str
    implementation_gate_open: bool


def _method_state(results: Mapping[str, object], method: str) -> tuple[str, str]:
    raw = results.get(method)
    if not isinstance(raw, Mapping):
        raise ValueError(f"missing method result: {method}")
    solve = raw.get("solve")
    proof = raw.get("proof")
    if solve not in _SOLVE_STATES:
        raise ValueError(f"invalid solve state for method {method}: {solve!r}")
    if proof not in _PROOF_STATES:
        raise ValueError(f"invalid proof state for method {method}: {proof!r}")
    if solve != "succeeded" and proof != "not_run":
        raise ValueError(f"method {method} cannot have proof={proof} after solve={solve}")
    return str(solve), str(proof)


def classify_comparison(document: Mapping[str, object]) -> Classification:
    """Classify one same-snapshot comparison without changing production.

    A result is successful only when both numerical solve and physical proof
    succeed. Local solver failure never constitutes a physical infeasibility
    certificate.
    """

    methods_raw = document.get("methods", document)
    if not isinstance(methods_raw, Mapping):
        raise ValueError("comparison methods must be an object")
    states = {name: _method_state(methods_raw, name) for name in "ABCD"}

    proof_failures = [
        method
        for method, (solve, proof) in states.items()
        if solve == "succeeded" and proof == "failed"
    ]
    if proof_failures:
        return Classification(
            "model_certificate_mismatch",
            "numerical solve succeeded but physical proof failed for "
            + ",".join(proof_failures),
            False,
        )

    succeeded = {
        method: solve == "succeeded" and proof == "accepted"
        for method, (solve, proof) in states.items()
    }
    attempted_failure = {
        method: solve == "failed" for method, (solve, _proof) in states.items()
    }

    if attempted_failure["A"] and succeeded["B"]:
        return Classification(
            "mission_lifecycle_defect",
            "persistent Mission failed while the same-SQP receding bundle succeeded",
            True,
        )
    if attempted_failure["A"] and attempted_failure["B"] and succeeded["C"]:
        return Classification(
            "candidate_generation_defect",
            "persistent and receding current candidates failed while a rough alternate candidate succeeded",
            True,
        )
    if all(attempted_failure[name] for name in "ABC") and succeeded["D"]:
        return Classification(
            "single_sqp_or_realtime_approximation_limit",
            "live candidate families failed while bounded offline refinement succeeded",
            True,
        )

    if all(attempted_failure.values()):
        certificate = document.get("physical_infeasibility_certificate")
        if isinstance(certificate, Mapping) and certificate.get("valid") is True:
            return Classification(
                "physical_infeasibility",
                "all methods failed and an explicit bounded physical infeasibility certificate is present",
                False,
            )
        return Classification(
            "unknown",
            "all attempted methods failed without a physical infeasibility certificate",
            False,
        )

    if document.get("offline_equivalent_succeeded") is True and all(
        attempted_failure[name] for name in "ABC"
    ):
        return Classification(
            "scheduling_or_lifecycle_defect",
            "an equivalent offline solve succeeded while live evaluations failed",
            True,
        )

    return Classification(
        "inconclusive",
        "comparison does not satisfy a registered causal classification",
        False,
    )
