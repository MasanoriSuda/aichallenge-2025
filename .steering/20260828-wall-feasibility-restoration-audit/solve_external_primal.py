#!/usr/bin/env python3
"""Solve one frozen QP with HiGHS and persist only its primal vector.

The output is consumed by the C++ audit verifier.  It has no production
authority and does not modify the recorded QP or solver configuration.
"""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path

import casadi as ca
import numpy as np


def load_conditioning_audit():
    source = Path(__file__).parents[1] / (
        "20260828-feasible-qp-conditioning-audit/analyze_conditioning.py"
    )
    spec = importlib.util.spec_from_file_location("conditioning_audit", source)
    if spec is None or spec.loader is None:
        raise RuntimeError("conditioning audit module unavailable")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()

    audit = load_conditioning_audit()
    _, p_upper, q, constraints, lower, upper, _ = audit.load(arguments.snapshot)
    quadratic = audit.symmetric_quadratic(p_upper)
    dense_constraints = constraints.toarray()
    structure = {
        "h": ca.DM(quadratic).sparsity(),
        "a": ca.DM(dense_constraints).sparsity(),
    }
    solver = ca.conic(
        "frozen_qp", "highs", structure, {"print_time": False}
    )
    solution = solver(
        h=ca.DM(quadratic),
        g=ca.DM(q),
        a=ca.DM(dense_constraints),
        lba=ca.DM(lower),
        uba=ca.DM(upper),
    )
    primal = np.asarray(solution["x"]).reshape(-1)
    values = constraints @ primal
    violation = np.maximum(lower - values, 0.0)
    violation = np.maximum(violation, np.maximum(values - upper, 0.0))
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(arguments.output, primal, fmt="%.17g")
    print(f"primal_output={arguments.output}")
    print(f"variable_count={primal.size}")
    print(f"max_physical_violation={violation.max():.12g}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
