#!/usr/bin/env python3
"""Eliminate exact equalities and solve the same frozen convex QP with OSQP."""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path

import numpy as np
import osqp
import scipy.linalg
import scipy.sparse


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


def solve(P, q, A, lower, upper, scaling: int):
    solver = osqp.OSQP()
    solver.setup(
        P=scipy.sparse.csc_matrix(np.triu(P)),
        q=q,
        A=scipy.sparse.csc_matrix(A),
        l=lower,
        u=upper,
        eps_abs=1.0e-3,
        eps_rel=0.0,
        max_iter=4000,
        scaling=scaling,
        warm_starting=False,
        polishing=False,
        verbose=False,
    )
    return solver.solve()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    arguments = parser.parse_args()
    audit = load_audit_module()
    _, P_upper, q, A_sparse, lower, upper, _ = audit.load(arguments.snapshot)
    P = audit.symmetric_quadratic(P_upper)
    A = A_sparse.toarray()

    equality = np.isfinite(lower) & np.isfinite(upper) & (lower == upper)
    E = A[equality]
    b = lower[equality]
    particular, _, rank, _ = np.linalg.lstsq(E, b, rcond=None)
    nullspace = scipy.linalg.null_space(E)
    equality_residual = np.max(np.abs(E @ particular - b))

    inequality = ~equality
    G = A[inequality] @ nullspace
    shift = A[inequality] @ particular
    condensed_lower = lower[inequality] - shift
    condensed_upper = upper[inequality] - shift
    physical_row_scale = audit.row_scales(lower, upper)[inequality]
    G = physical_row_scale[:, None] * G
    condensed_lower = physical_row_scale * condensed_lower
    condensed_upper = physical_row_scale * condensed_upper

    condensed_P = nullspace.T @ P @ nullspace
    condensed_P = 0.5 * (condensed_P + condensed_P.T)
    condensed_q = nullspace.T @ (P @ particular + q)

    print(f"snapshot={arguments.snapshot}")
    print(
        f"equality_rows={E.shape[0]}/rank:{rank}/"
        f"free_variables:{nullspace.shape[1]}/"
        f"particular_residual:{equality_residual:.12g}"
    )
    for scaling in (0, 10):
        result = solve(
            condensed_P,
            condensed_q,
            G,
            condensed_lower,
            condensed_upper,
            scaling,
        )
        print(
            f"condensed_scaling_{scaling}={result.info.status}/"
            f"iter:{result.info.iter}/prim:{result.info.prim_res:.9g}/"
            f"dual:{result.info.dual_res:.9g}"
        )
        if result.x is None:
            continue
        physical = particular + nullspace @ result.x
        values = A @ physical
        violation = np.maximum(lower - values, 0.0)
        violation = np.maximum(violation, np.maximum(values - upper, 0.0))
        objective = 0.5 * physical @ (P @ physical) + q @ physical
        print(
            f"condensed_physical_{scaling}=max_violation:"
            f"{violation.max():.12g}/objective:{objective:.12g}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
