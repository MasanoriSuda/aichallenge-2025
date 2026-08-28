#!/usr/bin/env python3
"""Reproduce and characterize one serialized feasible-but-unsolved MPCC QP.

This is an offline audit.  It never changes production solver settings or the
recorded payload.  The transformed problem follows PersistentOsqpSolver's
variable-coordinate and row-tolerance normalization exactly.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path

import numpy as np
import osqp
import scipy.optimize
import scipy.sparse
import yaml


ABSOLUTE_TOLERANCE = 1.0e-3
RELATIVE_TOLERANCE = 1.0e-3


def sparse_matrix(payload: dict) -> scipy.sparse.csc_matrix:
    triplets = payload["triplets"]
    rows = np.asarray([entry[0] for entry in triplets], dtype=int)
    columns = np.asarray([entry[1] for entry in triplets], dtype=int)
    values = np.asarray([entry[2] for entry in triplets], dtype=float)
    return scipy.sparse.coo_matrix(
        (values, (rows, columns)),
        shape=(int(payload["rows"]), int(payload["columns"])),
    ).tocsc()


def load(path: Path):
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    qp = payload["exact_qp"]
    return (
        payload,
        sparse_matrix(qp["quadratic_cost"]),
        np.asarray(qp["linear_cost"], dtype=float),
        sparse_matrix(qp["constraints"]),
        np.asarray(qp["lower_bound"], dtype=float),
        np.asarray(qp["upper_bound"], dtype=float),
        np.asarray(qp["variable_scaling"], dtype=float),
    )


def row_scales(lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    result = np.ones_like(lower)
    for row, (lo, hi) in enumerate(zip(lower, upper)):
        tolerance = np.inf
        if np.isfinite(lo):
            tolerance = min(
                tolerance,
                ABSOLUTE_TOLERANCE + RELATIVE_TOLERANCE * abs(lo),
            )
        if np.isfinite(hi):
            tolerance = min(
                tolerance,
                ABSOLUTE_TOLERANCE + RELATIVE_TOLERANCE * abs(hi),
            )
        if np.isfinite(tolerance):
            result[row] = ABSOLUTE_TOLERANCE / tolerance
    return result


def symmetric_quadratic(upper: scipy.sparse.csc_matrix) -> np.ndarray:
    dense = upper.toarray()
    return dense + dense.T - np.diag(np.diag(dense))


def solve_lp(
    constraints: scipy.sparse.csc_matrix,
    lower: np.ndarray,
    upper: np.ndarray,
):
    equality_mask = np.isfinite(lower) & np.isfinite(upper) & (lower == upper)
    lower_mask = np.isfinite(lower) & ~equality_mask
    upper_mask = np.isfinite(upper) & ~equality_mask
    inequalities = scipy.sparse.vstack(
        (-constraints[lower_mask], constraints[upper_mask]), format="csc"
    )
    inequality_upper = np.concatenate((-lower[lower_mask], upper[upper_mask]))
    return scipy.optimize.linprog(
        np.zeros(constraints.shape[1]),
        A_ub=inequalities,
        b_ub=inequality_upper,
        A_eq=constraints[equality_mask],
        b_eq=lower[equality_mask],
        bounds=[(None, None)] * constraints.shape[1],
        method="highs",
    )


def solve_python_osqp(
    quadratic: scipy.sparse.csc_matrix,
    linear: np.ndarray,
    constraints: scipy.sparse.csc_matrix,
    lower: np.ndarray,
    upper: np.ndarray,
    label: str,
    *,
    maximum_iterations: int = 4000,
    internal_scaling_iterations: int = 10,
):
    solver = osqp.OSQP()
    solver.setup(
        P=quadratic,
        q=linear,
        A=constraints,
        l=lower,
        u=upper,
        eps_abs=ABSOLUTE_TOLERANCE,
        eps_rel=0.0,
        max_iter=maximum_iterations,
        scaling=internal_scaling_iterations,
        warm_starting=False,
        polishing=False,
        verbose=False,
    )
    result = solver.solve()
    print(
        f"osqp_{label}=status:{result.info.status}/iter:{result.info.iter}/"
        f"prim:{result.info.prim_res:.9g}/dual:{result.info.dual_res:.9g}/"
        f"rho_updates:{result.info.rho_updates}"
    )
    return result


def row_layout(payload: dict) -> list[str]:
    request = payload["assembly_request"]
    horizon = int(request["horizon_steps"])
    state_dimension = 7
    input_dimension = 3
    state_values = state_dimension * (horizon + 1)
    input_values = input_dimension * horizon
    labels = []
    state_names = (
        "lateral",
        "lag",
        "heading",
        "velocity",
        "progress",
        "steering",
        "response-steering",
    )
    input_names = ("acceleration", "steering-rate", "progress-rate")
    for row in range(state_values):
        labels.append(f"dynamics/stage={row // 7}/{state_names[row % 7]}")
    for row in range(state_values):
        labels.append(f"state-box/stage={row // 7}/{state_names[row % 7]}")
    for row in range(input_values):
        labels.append(f"input-box/stage={row // 3}/{input_names[row % 3]}")
    if request.get("steering_rate_prefix_bounds") is not None:
        labels.extend(f"steering-prefix/stage={stage}" for stage in range(horizon))
    if request.get("progress_aligned_wall_constraints") is not None:
        for stage in range(horizon):
            labels.extend(
                (f"progress-wall-lower/stage={stage}", f"progress-wall-upper/stage={stage}")
            )
    labels.extend(
        f"swept-wall/index={index}"
        for index, _ in enumerate(request["swept_lateral_wall_constraints"])
    )
    for index, obstacle in enumerate(request["dynamic_obstacle_constraints"]):
        labels.append(
            f"dynamic/index={index}/stage={obstacle['state_stage']}/"
            f"axis={obstacle['axis']}"
        )
    return labels


def normalized_row_key(
    matrix: scipy.sparse.csr_matrix,
    row: int,
) -> tuple[tuple[tuple[int, float], ...], float]:
    start, end = matrix.indptr[row], matrix.indptr[row + 1]
    indices = matrix.indices[start:end]
    values = matrix.data[start:end]
    norm = np.max(np.abs(values)) if values.size else 1.0
    normalized = values / norm
    sign = -1.0 if normalized.size and normalized[0] < 0.0 else 1.0
    normalized *= sign
    return tuple(zip(indices.tolist(), np.round(normalized, 12).tolist())), sign / norm


def duplicate_expression_groups(
    constraints: scipy.sparse.csc_matrix,
) -> list[list[tuple[int, float]]]:
    csr = constraints.tocsr()
    groups: dict[tuple[tuple[int, float], ...], list[tuple[int, float]]] = defaultdict(list)
    for row in range(csr.shape[0]):
        key, bound_multiplier = normalized_row_key(csr, row)
        groups[key].append((row, bound_multiplier))
    return [rows for rows in groups.values() if len(rows) > 1]


def consolidate_duplicate_rows(
    constraints: scipy.sparse.csc_matrix,
    lower: np.ndarray,
    upper: np.ndarray,
):
    csr = constraints.tocsr()
    groups: dict[tuple[tuple[int, float], ...], list[tuple[int, float]]] = defaultdict(list)
    for row in range(csr.shape[0]):
        key, multiplier = normalized_row_key(csr, row)
        groups[key].append((row, multiplier))
    matrix_rows = []
    matrix_columns = []
    matrix_values = []
    consolidated_lower = []
    consolidated_upper = []
    removed = 0
    for output_row, (key, entries) in enumerate(groups.items()):
        lo = -np.inf
        hi = np.inf
        for row, multiplier in entries:
            row_lo = lower[row]
            row_hi = upper[row]
            if multiplier >= 0.0:
                normalized_lo = multiplier * row_lo
                normalized_hi = multiplier * row_hi
            else:
                normalized_lo = multiplier * row_hi
                normalized_hi = multiplier * row_lo
            lo = max(lo, normalized_lo)
            hi = min(hi, normalized_hi)
        for column, value in key:
            matrix_rows.append(output_row)
            matrix_columns.append(column)
            matrix_values.append(value)
        consolidated_lower.append(lo)
        consolidated_upper.append(hi)
        removed += len(entries) - 1
    return (
        scipy.sparse.coo_matrix(
            (matrix_values, (matrix_rows, matrix_columns)),
            shape=(len(groups), constraints.shape[1]),
        ).tocsc(),
        np.asarray(consolidated_lower),
        np.asarray(consolidated_upper),
        removed,
    )


def condition_report(name: str, matrix: np.ndarray) -> None:
    singular = np.linalg.svd(matrix, compute_uv=False)
    positive = singular[singular > max(matrix.shape) * np.finfo(float).eps * singular[0]]
    print(
        f"{name}=shape:{matrix.shape}/rank:{positive.size}/"
        f"sigma_max:{positive[0]:.9g}/sigma_min:{positive[-1]:.9g}/"
        f"condition:{positive[0] / positive[-1]:.9g}"
    )


def analyze(path: Path) -> int:
    payload, quadratic, linear, constraints, lower, upper, variable_scale = load(path)
    labels = row_layout(payload)
    if len(labels) != constraints.shape[0]:
        raise RuntimeError(f"row layout mismatch: {len(labels)} != {constraints.shape[0]}")

    lp = solve_lp(constraints, lower, upper)
    print(f"snapshot={path}")
    print(f"affine_feasible={int(lp.success)}/status:{lp.status}/{lp.message}")
    print(
        f"dimensions=variables:{constraints.shape[1]}/rows:{constraints.shape[0]}/"
        f"P_nnz:{quadratic.nnz}/A_nnz:{constraints.nnz}"
    )

    scale = row_scales(lower, upper)
    diagonal = scipy.sparse.diags(variable_scale)
    row_diagonal = scipy.sparse.diags(scale)
    solver_quadratic = (diagonal @ quadratic @ diagonal).tocsc()
    solver_linear = linear * variable_scale
    solver_constraints = (row_diagonal @ constraints @ diagonal).tocsc()
    solver_lower = lower * scale
    solver_upper = upper * scale

    print(
        f"variable_scale=min:{variable_scale.min():.9g}/max:{variable_scale.max():.9g}/"
        f"ratio:{variable_scale.max() / variable_scale.min():.9g}"
    )
    horizon = int(payload["assembly_request"]["horizon_steps"])
    state_names = (
        "lateral",
        "lag",
        "heading",
        "velocity",
        "progress",
        "steering",
        "response-steering",
    )
    input_names = ("acceleration", "steering-rate", "progress-rate")
    for element, name in enumerate(state_names):
        values = variable_scale[element : 7 * (horizon + 1) : 7]
        print(f"state_scale_{name}=min:{values.min():.9g}/max:{values.max():.9g}")
    input_offset = 7 * (horizon + 1)
    for element, name in enumerate(input_names):
        values = variable_scale[input_offset + element :: 3]
        print(f"input_scale_{name}=min:{values.min():.9g}/max:{values.max():.9g}")
    print(
        f"row_scale=min:{scale.min():.9g}/max:{scale.max():.9g}/"
        f"ratio:{scale.max() / scale.min():.9g}"
    )
    print(
        f"physical_A_abs=min:{np.abs(constraints.data).min():.9g}/"
        f"max:{np.abs(constraints.data).max():.9g}"
    )
    print(
        f"solver_A_abs=min:{np.abs(solver_constraints.data).min():.9g}/"
        f"max:{np.abs(solver_constraints.data).max():.9g}"
    )

    physical_hessian = symmetric_quadratic(quadratic)
    solver_hessian = symmetric_quadratic(solver_quadratic)
    condition_report("physical_hessian", physical_hessian)
    condition_report("solver_hessian", solver_hessian)
    equality = np.isfinite(lower) & np.isfinite(upper) & (lower == upper)
    condition_report("physical_equalities", constraints[equality].toarray())
    condition_report("solver_equalities", solver_constraints[equality].toarray())

    duplicate_groups = duplicate_expression_groups(constraints)
    print(
        f"duplicate_expression_groups={len(duplicate_groups)}/"
        f"duplicate_rows:{sum(len(group) - 1 for group in duplicate_groups)}"
    )
    for group in sorted(duplicate_groups, key=len, reverse=True)[:12]:
        print(
            "duplicate="
            + ";".join(
                f"row:{row}/{labels[row]}/bounds:[{lower[row]:.9g},{upper[row]:.9g}]"
                for row, _ in group
            )
        )

    baseline = solve_python_osqp(
        solver_quadratic,
        solver_linear,
        solver_constraints,
        solver_lower,
        solver_upper,
        "python_baseline",
    )
    solve_python_osqp(
        quadratic,
        linear,
        (row_diagonal @ constraints).tocsc(),
        solver_lower,
        solver_upper,
        "row_only_no_explicit_variable_scaling",
    )
    solve_python_osqp(
        solver_quadratic,
        solver_linear,
        solver_constraints,
        solver_lower,
        solver_upper,
        "python_baseline_50000",
        maximum_iterations=50000,
    )
    solve_python_osqp(
        solver_quadratic,
        solver_linear,
        solver_constraints,
        solver_lower,
        solver_upper,
        "python_baseline_no_internal_scaling",
        internal_scaling_iterations=0,
    )
    dedup_constraints, dedup_lower, dedup_upper, removed = consolidate_duplicate_rows(
        solver_constraints, solver_lower, solver_upper
    )
    print(f"deduplicated_rows_removed={removed}")
    deduplicated = solve_python_osqp(
        solver_quadratic,
        solver_linear,
        dedup_constraints,
        dedup_lower,
        dedup_upper,
        "python_deduplicated",
    )

    candidate = baseline.x if baseline.x is not None else deduplicated.x
    if candidate is not None:
        projected = solver_constraints @ candidate
        slack = np.minimum(projected - solver_lower, solver_upper - projected)
        finite = np.isfinite(slack)
        active = finite & (slack <= 2.0 * ABSOLUTE_TOLERANCE)
        active_matrix = solver_constraints[active].toarray()
        print(
            f"near_active_rows={int(active.sum())}/minimum_scaled_slack:"
            f"{slack[finite].min():.9g}"
        )
        if active_matrix.size:
            condition_report("solver_near_active", active_matrix)
        for row in np.argsort(np.where(finite, slack, np.inf))[:20]:
            print(
                f"tight_row={row}/{labels[row]}/slack:{slack[row]:.9g}/"
                f"bounds:[{solver_lower[row]:.9g},{solver_upper[row]:.9g}]/"
                f"value:{projected[row]:.9g}"
            )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    return analyze(parser.parse_args().snapshot)


if __name__ == "__main__":
    raise SystemExit(main())
