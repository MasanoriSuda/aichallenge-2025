#!/usr/bin/env python3
"""Compare deterministic explicit QP equilibrations with OSQP scaling off."""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path

import numpy as np
import osqp
import scipy.optimize
import scipy.sparse
import yaml


ABSOLUTE_TOLERANCE = 1.0e-3
RELATIVE_TOLERANCE = 1.0e-3
STATE_DIMENSION = 7
LAG_INDEX = 1


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


def relax_lag_bucket(payload, lower, upper, enabled):
    if not enabled:
        return lower.copy(), upper.copy()
    result_lower = lower.copy()
    result_upper = upper.copy()
    horizon = int(payload["assembly_request"]["horizon_steps"])
    state_values = STATE_DIMENSION * (horizon + 1)
    for stage in range(horizon + 1):
        row = state_values + stage * STATE_DIMENSION + LAG_INDEX
        result_lower[row] = -np.inf
        result_upper[row] = np.inf
    return result_lower, result_upper


def column_maximum(matrix):
    absolute = abs(matrix).tocsc()
    result = np.zeros(absolute.shape[1])
    for column in range(absolute.shape[1]):
        start, end = absolute.indptr[column], absolute.indptr[column + 1]
        if start != end:
            result[column] = absolute.data[start:end].max()
    return result


def row_maximum(matrix):
    return column_maximum(matrix.transpose().tocsc())


def limited_inverse_sqrt(values):
    safe = np.maximum(values, 1.0e-12)
    return np.clip(1.0 / np.sqrt(safe), 1.0e-3, 1.0e3)


def equilibrate(P, q, A, lower, upper, iterations):
    P_result = P.copy().tocsc()
    q_result = q.copy()
    A_result = A.copy().tocsc()
    lower_result = lower.copy()
    upper_result = upper.copy()
    variable_scale = np.ones(P.shape[0])
    row_scale = np.ones(A.shape[0])
    for _ in range(iterations):
        variable_metric = np.maximum(
            column_maximum(P_result), column_maximum(A_result)
        )
        variable_step = limited_inverse_sqrt(variable_metric)
        variable_diagonal = scipy.sparse.diags(variable_step)
        P_result = (variable_diagonal @ P_result @ variable_diagonal).tocsc()
        q_result *= variable_step
        A_result = (A_result @ variable_diagonal).tocsc()
        variable_scale *= variable_step

        constraint_step = limited_inverse_sqrt(row_maximum(A_result))
        row_diagonal = scipy.sparse.diags(constraint_step)
        A_result = (row_diagonal @ A_result).tocsc()
        lower_result *= constraint_step
        upper_result *= constraint_step
        row_scale *= constraint_step

    objective_norm = max(column_maximum(P_result).mean(), 1.0e-12)
    objective_scale = np.clip(1.0 / objective_norm, 1.0e-6, 1.0e6)
    P_result *= objective_scale
    q_result *= objective_scale
    return (
        P_result,
        q_result,
        A_result,
        lower_result,
        upper_result,
        variable_scale,
        row_scale,
        objective_scale,
    )


def physical_report(primal, P, q, A, lower, upper):
    values = A @ primal
    projected = np.minimum(np.maximum(values, lower), upper)
    violation = np.abs(values - projected)
    tolerance = ABSOLUTE_TOLERANCE + RELATIVE_TOLERANCE * np.maximum(
        np.abs(values), np.abs(projected)
    )
    normalized = violation / tolerance
    objective = 0.5 * primal @ (P @ primal) + q @ primal
    return objective, violation.max(), normalized.max(), int(np.argmax(normalized))


def solve(label, P, q, A, lower, upper, feasible):
    solver = osqp.OSQP()
    solver.setup(
        P=scipy.sparse.triu(P, format="csc"),
        q=q,
        A=A,
        l=lower,
        u=upper,
        eps_abs=ABSOLUTE_TOLERANCE,
        eps_rel=0.0,
        max_iter=4000,
        scaling=0,
        warm_starting=True,
        polishing=False,
        verbose=False,
    )
    solver.warm_start(x=feasible, y=np.zeros(A.shape[0]))
    result = solver.solve(raise_error=False)
    print(
        f"arm={label} status={result.info.status} iter={result.info.iter} "
        f"prim={result.info.prim_res:.12g} dual={result.info.dual_res:.12g}"
    )
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--omit-lag", action="store_true")
    arguments = parser.parse_args()
    audit = load_audit_module()
    payload, P_upper, q, A, lower, upper, box_scale = audit.load(arguments.snapshot)
    P = scipy.sparse.csc_matrix(audit.symmetric_quadratic(P_upper))
    lower, upper = relax_lag_bucket(
        payload, lower, upper, arguments.omit_lag
    )
    feasible_lp = audit.solve_lp(A, lower, upper)
    print(
        f"snapshot={arguments.snapshot} omit_lag={int(arguments.omit_lag)} "
        f"affine_feasible={int(feasible_lp.success)}"
    )
    if not feasible_lp.success:
        return 4
    physical_feasible = np.asarray(feasible_lp.x)
    tolerance_row_scale = audit.row_scales(lower, upper)
    base_variable = scipy.sparse.diags(box_scale)
    base_rows = scipy.sparse.diags(tolerance_row_scale)
    base_P = (base_variable @ P @ base_variable).tocsc()
    base_q = q * box_scale
    base_A = (base_rows @ A @ base_variable).tocsc()
    base_lower = lower * tolerance_row_scale
    base_upper = upper * tolerance_row_scale
    base_feasible = physical_feasible / box_scale

    variants = [("box-row", base_P, base_q, base_A, base_lower, base_upper,
                 np.ones_like(box_scale), 1.0)]
    for iterations in (1, 3, 10):
        result = equilibrate(
            base_P, base_q, base_A, base_lower, base_upper, iterations
        )
        variants.append(
            (f"explicit-ruiz-{iterations}", result[0], result[1], result[2],
             result[3], result[4], result[5], result[7])
        )
    for label, transformed_P, transformed_q, transformed_A, transformed_lower, transformed_upper, extra_variable, objective_scale in variants:
        result = solve(
            label, transformed_P, transformed_q, transformed_A,
            transformed_lower, transformed_upper, base_feasible / extra_variable
        )
        if result.x is None:
            continue
        physical = result.x * extra_variable * box_scale
        objective, violation, normalized, row = physical_report(
            physical, P, q, A, lower, upper
        )
        print(
            f"arm={label} objective={objective:.12g} "
            f"violation={violation:.12g} normalized={normalized:.12g} "
            f"row={row} extra_variable_min={extra_variable.min():.9g} "
            f"extra_variable_max={extra_variable.max():.9g} "
            f"objective_scale={objective_scale:.9g}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
