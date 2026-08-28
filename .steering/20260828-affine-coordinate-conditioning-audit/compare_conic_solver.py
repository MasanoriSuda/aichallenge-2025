#!/usr/bin/env python3
"""Solve one frozen physical QP with an independent CasADi conic backend."""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import time

import casadi as ca
import numpy as np


def load_audit_module():
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
    parser.add_argument("plugin")
    arguments = parser.parse_args()

    audit = load_audit_module()
    _, P_upper, q, A, lower, upper, _ = audit.load(arguments.snapshot)
    P = audit.symmetric_quadratic(P_upper)
    h = ca.DM(P)
    a = ca.DM(A.toarray())
    structure = {"h": h.sparsity(), "a": a.sparsity()}
    options = {"print_time": False}
    if arguments.plugin == "qpoases":
        options["printLevel"] = "none"
    start = time.perf_counter()
    solver = ca.conic("frozen_qp", arguments.plugin, structure, options)
    solution = solver(
        h=h,
        g=ca.DM(q),
        a=a,
        lba=ca.DM(lower),
        uba=ca.DM(upper),
    )
    elapsed_ms = 1000.0 * (time.perf_counter() - start)
    primal = np.asarray(solution["x"]).reshape(-1)
    values = A @ primal
    violation = np.maximum(lower - values, 0.0)
    violation = np.maximum(violation, np.maximum(values - upper, 0.0))
    objective = 0.5 * primal @ (P @ primal) + q @ primal
    print(f"snapshot={arguments.snapshot}")
    print(f"plugin={arguments.plugin}")
    print(f"elapsed_ms={elapsed_ms:.6f}")
    print(f"max_physical_violation={violation.max():.12g}")
    print(f"objective={objective:.12g}")
    print(f"stats={solver.stats()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
