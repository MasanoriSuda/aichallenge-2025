#!/usr/bin/env python3
"""Compare numerical backends on one bucket-relaxed frozen racing QP."""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import time

import casadi as ca
import numpy as np
import osqp
import scipy.optimize
import scipy.sparse


ABSOLUTE_TOLERANCE = 1.0e-3
RELATIVE_TOLERANCE = 1.0e-3
STATE_DIMENSION = 7
LAG_INDEX = 1
HEADING_INDEX = 2


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


def relaxed_bounds(payload, lower, upper, bucket: str):
    result_lower = lower.copy()
    result_upper = upper.copy()
    horizon = int(payload["assembly_request"]["horizon_steps"])
    state_values = STATE_DIMENSION * (horizon + 1)
    state_index = LAG_INDEX if bucket == "lag" else HEADING_INDEX
    rows = []
    for stage in range(horizon + 1):
        row = state_values + stage * STATE_DIMENSION + state_index
        result_lower[row] = -np.inf
        result_upper[row] = np.inf
        rows.append(row)
    return result_lower, result_upper, rows


def physical_report(label, primal, P, q, A, lower, upper, elapsed_ms):
    values = A @ primal
    projected = np.minimum(np.maximum(values, lower), upper)
    violation = np.abs(values - projected)
    scale = np.maximum(np.abs(values), np.abs(projected))
    tolerance = ABSOLUTE_TOLERANCE + RELATIVE_TOLERANCE * scale
    normalized = violation / tolerance
    objective = 0.5 * primal @ (P @ primal) + q @ primal
    worst = int(np.argmax(normalized))
    print(
        f"arm={label} elapsed_ms={elapsed_ms:.6f} "
        f"objective={objective:.12g} max_violation={violation.max():.12g} "
        f"max_normalized={normalized[worst]:.12g} worst_row={worst}"
    )
    return normalized[worst] <= 1.0


def solve_osqp(
    label,
    P,
    q,
    A,
    lower,
    upper,
    *,
    internal_scaling,
    relative_tolerance,
    feasible_primal=None,
):
    solver = osqp.OSQP()
    solver.setup(
        P=scipy.sparse.triu(P, format="csc"),
        q=q,
        A=A,
        l=lower,
        u=upper,
        eps_abs=ABSOLUTE_TOLERANCE,
        eps_rel=relative_tolerance,
        max_iter=4000,
        scaling=internal_scaling,
        warm_starting=feasible_primal is not None,
        polishing=False,
        verbose=False,
    )
    if feasible_primal is not None:
        solver.warm_start(x=feasible_primal, y=np.zeros(A.shape[0]))
    started = time.perf_counter()
    result = solver.solve(raise_error=False)
    elapsed_ms = 1000.0 * (time.perf_counter() - started)
    print(
        f"solver={label} status={result.info.status} iter={result.info.iter} "
        f"prim={result.info.prim_res:.12g} dual={result.info.dual_res:.12g} "
        f"rho_updates={result.info.rho_updates} solve_ms={elapsed_ms:.6f}"
    )
    return result.x, elapsed_ms, result.info.status


def solve_conic(label, P, q, A, lower, upper):
    structure = {"h": ca.DM(P).sparsity(), "a": ca.DM(A.toarray()).sparsity()}
    options = {"print_time": False}
    if label == "qpoases":
        options["printLevel"] = "none"
    started = time.perf_counter()
    solver = ca.conic(f"bucket_{label}", label, structure, options)
    solution = solver(
        h=ca.DM(P),
        g=ca.DM(q),
        a=ca.DM(A.toarray()),
        lba=ca.DM(lower),
        uba=ca.DM(upper),
    )
    elapsed_ms = 1000.0 * (time.perf_counter() - started)
    primal = np.asarray(solution["x"]).reshape(-1)
    print(f"solver={label} stats={solver.stats()}")
    return primal, elapsed_ms


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--bucket", choices=("lag", "heading"), default="lag")
    parser.add_argument("--output-dir", type=Path)
    arguments = parser.parse_args()

    audit = load_audit_module()
    payload, P_upper, q, A, lower, upper, variable_scale = audit.load(
        arguments.snapshot
    )
    P = audit.symmetric_quadratic(P_upper)
    lower, upper, relaxed_rows = relaxed_bounds(
        payload, lower, upper, arguments.bucket
    )
    print(f"snapshot={arguments.snapshot}")
    print(f"bucket={arguments.bucket} relaxed_rows={relaxed_rows}")

    feasible = audit.solve_lp(A, lower, upper)
    print(f"affine_feasible={int(feasible.success)} status={feasible.message}")
    if not feasible.success:
        return 4
    feasible_physical = np.asarray(feasible.x)

    row_scale = audit.row_scales(lower, upper)
    variable_diagonal = scipy.sparse.diags(variable_scale)
    row_diagonal = scipy.sparse.diags(row_scale)
    explicit_P = (variable_diagonal @ scipy.sparse.csc_matrix(P) @ variable_diagonal).tocsc()
    explicit_q = q * variable_scale
    explicit_A = (row_diagonal @ A @ variable_diagonal).tocsc()
    explicit_lower = lower * row_scale
    explicit_upper = upper * row_scale
    feasible_explicit = feasible_physical / variable_scale

    arms = {}
    for label, internal_scaling in (("explicit-osqp-0", 0), ("explicit-osqp-10", 10)):
        primal, elapsed_ms, _ = solve_osqp(
            label,
            explicit_P,
            explicit_q,
            explicit_A,
            explicit_lower,
            explicit_upper,
            internal_scaling=internal_scaling,
            relative_tolerance=0.0,
            feasible_primal=feasible_explicit,
        )
        if primal is not None:
            physical = primal * variable_scale
            physical_report(label, physical, P, q, A, lower, upper, elapsed_ms)
            arms[label] = physical

    primal, elapsed_ms, _ = solve_osqp(
        "raw-osqp-10",
        scipy.sparse.csc_matrix(P),
        q,
        A,
        lower,
        upper,
        internal_scaling=10,
        relative_tolerance=RELATIVE_TOLERANCE,
        feasible_primal=feasible_physical,
    )
    if primal is not None:
        physical_report("raw-osqp-10", primal, P, q, A, lower, upper, elapsed_ms)
        arms["raw-osqp-10"] = primal

    for plugin in ("qpoases", "proxqp", "highs"):
        try:
            primal, elapsed_ms = solve_conic(plugin, P, q, A, lower, upper)
        except Exception as error:  # backend availability is environment-specific
            print(f"solver={plugin} exception={type(error).__name__}:{error}")
            continue
        physical_report(plugin, primal, P, q, A, lower, upper, elapsed_ms)
        arms[plugin] = primal

    if arguments.output_dir is not None:
        arguments.output_dir.mkdir(parents=True, exist_ok=True)
        for label, primal in arms.items():
            np.savetxt(arguments.output_dir / f"{label}.txt", primal)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
