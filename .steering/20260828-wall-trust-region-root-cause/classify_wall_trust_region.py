#!/usr/bin/env python3
"""Classify refinement-owned infeasibility in a frozen seven-state QP.

This is observation-only tooling. It ignores the objective and production
solver settings, preserves dynamics exactly, and uses HiGHS only to identify
which hard-constraint families make the recorded affine feasible set empty.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import scipy.optimize
import scipy.sparse
import yaml


STATE_DIMENSION = 7
INPUT_DIMENSION = 3
STATE_NAMES = (
    "lateral",
    "lag",
    "heading",
    "velocity",
    "progress",
    "steering",
    "response-steering",
)
INPUT_NAMES = ("acceleration", "steering-rate", "progress-rate")


def load(path: Path):
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    request = payload["assembly_request"]
    exact = payload["exact_qp"]
    matrix = exact["constraints"]
    rows = np.asarray([item[0] for item in matrix["triplets"]], dtype=int)
    columns = np.asarray([item[1] for item in matrix["triplets"]], dtype=int)
    values = np.asarray([item[2] for item in matrix["triplets"]], dtype=float)
    constraints = scipy.sparse.coo_matrix(
        (values, (rows, columns)),
        shape=(int(matrix["rows"]), int(matrix["columns"])),
    ).tocsr()
    lower = np.asarray(exact["lower_bound"], dtype=float)
    upper = np.asarray(exact["upper_bound"], dtype=float)
    warm = payload.get("warm_start", {})
    warm_primal = None
    if warm.get("available", False):
        warm_primal = np.asarray(warm["primal"], dtype=float)
    return payload, request, constraints, lower, upper, warm_primal


def groups(request: dict, row_count: int) -> tuple[dict[str, np.ndarray], list[str]]:
    horizon = int(request["horizon_steps"])
    state_values = STATE_DIMENSION * (horizon + 1)
    input_values = INPUT_DIMENSION * horizon
    state_box_offset = state_values
    input_box_offset = state_box_offset + state_values
    steering_offset = input_box_offset + input_values
    progress_wall_offset = steering_offset + horizon
    swept_wall_offset = progress_wall_offset + 2 * horizon
    swept_count = len(request.get("swept_lateral_wall_constraints", []))
    dynamic_offset = swept_wall_offset + swept_count
    dynamic_count = len(request.get("dynamic_obstacle_constraints", []))

    state_rows = np.arange(state_box_offset, input_box_offset).reshape(
        horizon + 1, STATE_DIMENSION
    )
    input_rows = np.arange(input_box_offset, steering_offset).reshape(
        horizon, INPUT_DIMENSION
    )
    result: dict[str, np.ndarray] = {
        "dynamics": np.arange(0, state_values),
        "state-box": np.arange(state_box_offset, input_box_offset),
        "input-box": np.arange(input_box_offset, steering_offset),
        "steering-prefix": np.arange(steering_offset, progress_wall_offset),
        "progress-wall": np.arange(progress_wall_offset, swept_wall_offset),
        "swept-wall": np.arange(swept_wall_offset, dynamic_offset),
        "dynamic-obstacle": np.arange(dynamic_offset, dynamic_offset + dynamic_count),
    }
    for index, name in enumerate(STATE_NAMES):
        result[f"state-{name}"] = state_rows[:, index]
    for index, name in enumerate(INPUT_NAMES):
        result[f"input-{name}"] = input_rows[:, index]
    result["explicit-wall"] = np.concatenate(
        (result["progress-wall"], result["swept-wall"])
    )
    result["refinement-pose-box"] = np.concatenate(
        (
            result["state-lateral"],
            result["state-lag"],
            result["state-heading"],
            result["state-progress"],
        )
    )
    result["physical-wall-envelope"] = np.concatenate(
        (result["refinement-pose-box"], result["explicit-wall"])
    )
    if dynamic_offset + dynamic_count != row_count:
        raise ValueError(
            f"decoded rows {dynamic_offset + dynamic_count} != {row_count}"
        )

    labels = ["unknown"] * row_count
    for row in result["dynamics"]:
        labels[int(row)] = f"dynamics[{int(row) // STATE_DIMENSION}].{STATE_NAMES[int(row) % STATE_DIMENSION]}"
    for state in range(horizon + 1):
        for element, name in enumerate(STATE_NAMES):
            labels[int(state_rows[state, element])] = f"state[{state}].{name}"
    for stage in range(horizon):
        for element, name in enumerate(INPUT_NAMES):
            labels[int(input_rows[stage, element])] = f"input[{stage}].{name}"
        labels[steering_offset + stage] = f"steering-prefix[{stage}]"
        labels[progress_wall_offset + 2 * stage] = f"progress-wall-lower[{stage}]"
        labels[progress_wall_offset + 2 * stage + 1] = f"progress-wall-upper[{stage}]"
    for index, wall in enumerate(request.get("swept_lateral_wall_constraints", [])):
        labels[swept_wall_offset + index] = (
            f"swept-wall[{int(wall['transition_stage'])},"
            f"{float(wall['destination_ratio']):.3f}]"
        )
    for index in range(dynamic_count):
        labels[dynamic_offset + index] = f"dynamic-obstacle[{index}]"
    return result, labels


def solve(
    constraints: scipy.sparse.csr_matrix,
    lower: np.ndarray,
    upper: np.ndarray,
    keep: np.ndarray,
):
    matrix = constraints[keep]
    selected_lower = lower[keep]
    selected_upper = upper[keep]
    equality = (
        np.isfinite(selected_lower)
        & np.isfinite(selected_upper)
        & np.isclose(selected_lower, selected_upper, atol=0.0, rtol=0.0)
    )
    lower_only = np.isfinite(selected_lower) & ~equality
    upper_only = np.isfinite(selected_upper) & ~equality
    inequality = scipy.sparse.vstack(
        (-matrix[lower_only], matrix[upper_only]), format="csr"
    )
    inequality_bound = np.concatenate(
        (-selected_lower[lower_only], selected_upper[upper_only])
    )
    return scipy.optimize.linprog(
        np.zeros(constraints.shape[1]),
        A_ub=inequality,
        b_ub=inequality_bound,
        A_eq=matrix[equality],
        b_eq=selected_lower[equality],
        bounds=[(None, None)] * constraints.shape[1],
        method="highs",
    )


def minimum_group_slack(
    constraints: scipy.sparse.csr_matrix,
    lower: np.ndarray,
    upper: np.ndarray,
    selected_rows: np.ndarray,
):
    selected = np.zeros(constraints.shape[0], dtype=bool)
    selected[selected_rows] = True
    variable_count = constraints.shape[1]
    inequality_rows: list[scipy.sparse.csr_matrix] = []
    inequality_bounds: list[float] = []
    equality_rows: list[scipy.sparse.csr_matrix] = []
    equality_bounds: list[float] = []
    for row in range(constraints.shape[0]):
        coefficients = constraints.getrow(row)
        finite_lower = math.isfinite(float(lower[row]))
        finite_upper = math.isfinite(float(upper[row]))
        fixed = finite_lower and finite_upper and lower[row] == upper[row]
        if fixed and not selected[row]:
            equality_rows.append(
                scipy.sparse.hstack((coefficients, [[0.0]]), format="csr")
            )
            equality_bounds.append(float(lower[row]))
            continue
        slack_coefficient = -1.0 if selected[row] else 0.0
        if finite_upper:
            inequality_rows.append(
                scipy.sparse.hstack(
                    (coefficients, [[slack_coefficient]]), format="csr"
                )
            )
            inequality_bounds.append(float(upper[row]))
        if finite_lower:
            inequality_rows.append(
                scipy.sparse.hstack(
                    (-coefficients, [[slack_coefficient]]), format="csr"
                )
            )
            inequality_bounds.append(float(-lower[row]))
    cost = np.zeros(variable_count + 1)
    cost[-1] = 1.0
    return scipy.optimize.linprog(
        cost,
        A_ub=scipy.sparse.vstack(inequality_rows, format="csr"),
        b_ub=np.asarray(inequality_bounds),
        A_eq=(
            scipy.sparse.vstack(equality_rows, format="csr")
            if equality_rows
            else None
        ),
        b_eq=np.asarray(equality_bounds) if equality_rows else None,
        bounds=[(None, None)] * variable_count + [(0.0, None)],
        method="highs",
    )


def audit(path: Path) -> int:
    payload, request, constraints, lower, upper, warm = load(path)
    decoded, labels = groups(request, constraints.shape[0])
    keep_all = np.ones(constraints.shape[0], dtype=bool)
    exact = solve(constraints, lower, upper, keep_all)
    print(f"snapshot={path}")
    print(f"pipeline_stage={payload['pipeline_stage']}")
    print(f"intent={payload['source']['problem_context']['intent']}")
    print(f"exact_affine_feasible={int(exact.success)}")
    print(f"exact_affine_status={exact.message}")

    for name in (
        "state-heading",
        "state-lag",
        "state-progress",
        "state-lateral",
        "steering-prefix",
        "explicit-wall",
        "refinement-pose-box",
        "physical-wall-envelope",
    ):
        keep = keep_all.copy()
        keep[decoded[name]] = False
        result = solve(constraints, lower, upper, keep)
        print(f"feasible_without_{name}={int(result.success)}")

    for name in (
        "state-heading",
        "state-lag",
        "state-progress",
        "state-lateral",
        "steering-prefix",
        "refinement-pose-box",
    ):
        result = minimum_group_slack(
            constraints, lower, upper, decoded[name]
        )
        value = float(result.fun) if result.success else math.nan
        print(
            f"minimum_{name}_slack={value:.9f} "
            f"selective_slack_feasible={int(result.success)}"
        )

    if warm is not None and warm.shape == (constraints.shape[1],):
        projected = np.asarray(constraints @ warm).reshape(-1)
        violation = np.zeros(constraints.shape[0])
        finite_lower = np.isfinite(lower)
        finite_upper = np.isfinite(upper)
        violation[finite_lower] = np.maximum(
            violation[finite_lower], lower[finite_lower] - projected[finite_lower]
        )
        violation[finite_upper] = np.maximum(
            violation[finite_upper], projected[finite_upper] - upper[finite_upper]
        )
        for row in np.argsort(violation)[::-1][:16]:
            if violation[row] <= 1e-12:
                break
            print(
                f"warm_violation={violation[row]:.9f} row={row} "
                f"constraint={labels[row]} value={projected[row]:.9f} "
                f"bounds=[{lower[row]:.9f},{upper[row]:.9f}]"
            )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    return audit(parser.parse_args().snapshot)


if __name__ == "__main__":
    raise SystemExit(main())
