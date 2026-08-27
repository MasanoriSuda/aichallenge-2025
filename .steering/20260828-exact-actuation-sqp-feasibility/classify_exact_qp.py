#!/usr/bin/env python3
"""Classify mathematical feasibility of one serialized exact MPCC QP.

The objective and solver warm start are deliberately ignored.  The script
asks whether the original physical-coordinate A/l/u rows have any common
solution, which separates QP infeasibility from OSQP convergence/lifecycle
failures without changing production settings.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import scipy.optimize
import scipy.sparse
import yaml


def load_exact_qp(path: Path):
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    qp = payload["exact_qp"]
    matrix = qp["constraints"]
    rows = int(matrix["rows"])
    columns = int(matrix["columns"])
    triplets = matrix["triplets"]
    row = np.asarray([entry[0] for entry in triplets], dtype=int)
    column = np.asarray([entry[1] for entry in triplets], dtype=int)
    value = np.asarray([entry[2] for entry in triplets], dtype=float)
    constraints = scipy.sparse.coo_matrix(
        (value, (row, column)), shape=(rows, columns)
    ).tocsr()
    lower = np.asarray(qp["lower_bound"], dtype=float)
    upper = np.asarray(qp["upper_bound"], dtype=float)
    return payload, constraints, lower, upper


def solve_rows(constraints, lower, upper, keep_rows):
    constraints = constraints[keep_rows]
    lower = lower[keep_rows]
    upper = upper[keep_rows]
    equality_mask = np.isfinite(lower) & np.isfinite(upper) & (lower == upper)
    lower_mask = np.isfinite(lower) & ~equality_mask
    upper_mask = np.isfinite(upper) & ~equality_mask

    equality = constraints[equality_mask]
    equality_value = lower[equality_mask]
    inequality = scipy.sparse.vstack(
        (-constraints[lower_mask], constraints[upper_mask]), format="csr"
    )
    inequality_upper = np.concatenate((-lower[lower_mask], upper[upper_mask]))

    return scipy.optimize.linprog(
        np.zeros(constraints.shape[1]),
        A_ub=inequality,
        b_ub=inequality_upper,
        A_eq=equality,
        b_eq=equality_value,
        bounds=[(None, None)] * constraints.shape[1],
        method="highs",
    )


def row_groups(payload):
    request = payload["assembly_request"]
    horizon = int(request["horizon_steps"])
    state_dimension = 7
    input_dimension = 3
    state_values = state_dimension * (horizon + 1)
    input_values = input_dimension * horizon
    box_offset = state_values
    steering_offset = box_offset + state_values + input_values
    progress_wall_offset = steering_offset + horizon
    swept_wall_offset = progress_wall_offset + 2 * horizon
    swept_wall_count = len(request["swept_lateral_wall_constraints"])
    dynamic_offset = swept_wall_offset + swept_wall_count
    dynamic_count = len(request["dynamic_obstacle_constraints"])
    groups = {
        "dynamics": np.arange(0, state_values),
        "state-box": np.arange(box_offset, box_offset + state_values),
        "input-box": np.arange(
            box_offset + state_values, box_offset + state_values + input_values
        ),
        "steering-prefix": np.arange(steering_offset, progress_wall_offset),
        "progress-wall": np.arange(progress_wall_offset, swept_wall_offset),
        "swept-wall": np.arange(swept_wall_offset, dynamic_offset),
        "dynamic-obstacle": np.arange(dynamic_offset, dynamic_offset + dynamic_count),
    }
    state_box_rows = np.arange(box_offset, box_offset + state_values).reshape(
        horizon + 1, state_dimension
    )
    input_box_rows = np.arange(
        box_offset + state_values, box_offset + state_values + input_values
    ).reshape(horizon, input_dimension)
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
    for index, name in enumerate(state_names):
        groups[f"state-{name}"] = state_box_rows[:, index]
    for index, name in enumerate(input_names):
        groups[f"input-{name}"] = input_box_rows[:, index]
    groups["all-wall"] = np.concatenate(
        (groups["progress-wall"], groups["swept-wall"])
    )
    groups["all-steering-limits"] = np.concatenate(
        (
            groups["state-steering"],
            groups["state-response-steering"],
            groups["input-steering-rate"],
            groups["steering-prefix"],
        )
    )
    groups["all-longitudinal-limits"] = np.concatenate(
        (groups["state-velocity"], groups["input-acceleration"])
    )
    groups["all-progress-limits"] = np.concatenate(
        (
            groups["state-progress"],
            groups["input-progress-rate"],
            groups["progress-wall"],
        )
    )
    return groups


def classify(path: Path, diagnose_groups: bool) -> int:
    payload, constraints, lower, upper = load_exact_qp(path)
    keep_all = np.ones(constraints.shape[0], dtype=bool)
    result = solve_rows(constraints, lower, upper, keep_all)
    print(f"snapshot={path}")
    print(f"pipeline_stage={payload['pipeline_stage']}")
    print(f"failure_outcome={payload['failure_outcome']}")
    print(f"lp_status={result.status}")
    print(f"lp_message={result.message}")
    print(f"mathematically_feasible={int(result.success)}")
    if result.success:
        projected = constraints @ result.x
        violation = np.zeros_like(projected)
        finite_lower = np.isfinite(lower)
        finite_upper = np.isfinite(upper)
        violation[finite_lower] = np.maximum(
            violation[finite_lower], lower[finite_lower] - projected[finite_lower]
        )
        violation[finite_upper] = np.maximum(
            violation[finite_upper], projected[finite_upper] - upper[finite_upper]
        )
        worst = int(np.argmax(violation))
        print(f"maximum_lp_row_violation={violation[worst]:.17g}")
        print(f"maximum_lp_row={worst}")
    if diagnose_groups and not result.success:
        for name, rows in row_groups(payload).items():
            if rows.size == 0:
                continue
            keep = keep_all.copy()
            keep[rows] = False
            without_group = solve_rows(constraints, lower, upper, keep)
            print(f"feasible_without_{name}={int(without_group.success)}")
    return 0 if result.success else 2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--diagnose-groups", action="store_true")
    arguments = parser.parse_args()
    return classify(arguments.snapshot, arguments.diagnose_groups)


if __name__ == "__main__":
    raise SystemExit(main())
