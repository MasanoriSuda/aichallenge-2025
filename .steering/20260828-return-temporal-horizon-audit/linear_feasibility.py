#!/usr/bin/env python3
"""Independent feasibility audit for a recorded seven-state MPCC QP.

This tool deliberately ignores the recorded quadratic objective and production
OSQP configuration.  It reconstructs the immutable affine hard constraints and
asks SciPy HiGHS whether their feasible set is empty.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from scipy.optimize import linprog


NX = 7
NU = 3
LATERAL = 0
LAG = 1
PROGRESS = 4


def _matrix(node: dict[str, Any]) -> np.ndarray:
    values = np.asarray(node["values"], dtype=float)
    expected = (int(node["rows"]), int(node["columns"]))
    if values.shape != expected:
        raise ValueError(f"matrix shape {values.shape} != {expected}")
    return values


def _bounded_rows(
    coefficients: np.ndarray,
    lower: float,
    upper: float,
    a_ub: list[np.ndarray],
    b_ub: list[float],
    labels: list[str],
    label: str,
) -> None:
    if math.isfinite(upper):
        a_ub.append(coefficients.copy())
        b_ub.append(upper)
        labels.append(f"{label}.upper")
    if math.isfinite(lower):
        a_ub.append(-coefficients.copy())
        b_ub.append(-lower)
        labels.append(f"{label}.lower")


def solve(snapshot_path: Path) -> int:
    with snapshot_path.open("r", encoding="utf-8") as stream:
        snapshot = yaml.safe_load(stream)
    request = snapshot["assembly_request"]
    horizon = int(request["horizon_steps"])
    if horizon <= 0:
        raise ValueError("non-positive horizon")

    state_count = NX * (horizon + 1)
    variable_count = state_count + NU * horizon
    initial = np.asarray(request["initial_state"], dtype=float)
    state_lower = np.asarray(request["state_lower"], dtype=float)
    state_upper = np.asarray(request["state_upper"], dtype=float)
    input_lower = np.asarray(request["input_lower"], dtype=float)
    input_upper = np.asarray(request["input_upper"], dtype=float)
    if initial.shape != (NX,):
        raise ValueError("initial state shape mismatch")
    if state_lower.shape != (state_count,) or state_upper.shape != (state_count,):
        raise ValueError("state bound shape mismatch")
    if input_lower.shape != (NU * horizon,) or input_upper.shape != (NU * horizon,):
        raise ValueError("input bound shape mismatch")

    a_eq: list[np.ndarray] = []
    b_eq: list[float] = []
    for element in range(NX):
        row = np.zeros(variable_count)
        row[element] = 1.0
        a_eq.append(row)
        b_eq.append(float(initial[element]))

    linearizations = request["linearizations"]
    if len(linearizations) != horizon:
        raise ValueError("linearization count mismatch")
    stage_dt: list[float] = []
    for stage, node in enumerate(linearizations):
        state_matrix = _matrix(node["state_matrix"])
        input_matrix = _matrix(node["input_matrix"])
        offset = np.asarray(node["equality_offset"], dtype=float)
        if state_matrix.shape != (NX, NX) or input_matrix.shape != (NX, NU):
            raise ValueError("linearization dimension mismatch")
        if offset.shape != (NX,):
            raise ValueError("equality offset shape mismatch")
        stage_dt.append(float(node["stage_dt_sec"]))
        for element in range(NX):
            row = np.zeros(variable_count)
            row[stage * NX : (stage + 1) * NX] = state_matrix[element]
            input_offset = state_count + stage * NU
            row[input_offset : input_offset + NU] = input_matrix[element]
            row[(stage + 1) * NX + element] = -1.0
            a_eq.append(row)
            b_eq.append(float(offset[element]))

    a_ub: list[np.ndarray] = []
    b_ub: list[float] = []
    inequality_labels: list[str] = []

    steering_prefix = request.get("steering_rate_prefix_bounds")
    if steering_prefix is not None:
        for stage in range(horizon):
            row = np.zeros(variable_count)
            for prefix_stage in range(stage + 1):
                row[state_count + prefix_stage * NU + 1] = stage_dt[prefix_stage]
            _bounded_rows(
                row,
                float(steering_prefix["minimum_cumulative_delta_rad"]),
                float(steering_prefix["maximum_cumulative_delta_rad"]),
                a_ub,
                b_ub,
                inequality_labels,
                f"steering_prefix[{stage}]",
            )

    progress_wall = request.get("progress_aligned_wall_constraints")
    if progress_wall is not None:
        for stage in range(horizon):
            state = (stage + 1) * NX
            lower_row = np.zeros(variable_count)
            lower_row[state + LATERAL] = 1.0
            lower_row[state + PROGRESS] = -float(progress_wall["lower_slope"][stage])
            _bounded_rows(
                lower_row,
                float(progress_wall["lower_intercept"][stage]),
                math.inf,
                a_ub,
                b_ub,
                inequality_labels,
                f"progress_wall_lower[{stage}]",
            )
            upper_row = np.zeros(variable_count)
            upper_row[state + LATERAL] = 1.0
            upper_row[state + PROGRESS] = -float(progress_wall["upper_slope"][stage])
            _bounded_rows(
                upper_row,
                -math.inf,
                float(progress_wall["upper_intercept"][stage]),
                a_ub,
                b_ub,
                inequality_labels,
                f"progress_wall_upper[{stage}]",
            )

    for wall in request.get("swept_lateral_wall_constraints", []):
        transition = int(wall["transition_stage"])
        ratio = float(wall["destination_ratio"])
        row = np.zeros(variable_count)
        row[transition * NX + LATERAL] = 1.0 - ratio
        row[(transition + 1) * NX + LATERAL] = ratio
        _bounded_rows(
            row,
            float(wall["lower_m"]),
            float(wall["upper_m"]),
            a_ub,
            b_ub,
            inequality_labels,
            f"swept_wall[{transition},{ratio:.3f}]",
        )

    dynamic_count = 0
    for obstacle in request.get("dynamic_obstacle_constraints", []):
        dynamic_count += 1
        stage = int(obstacle["state_stage"])
        row = np.zeros(variable_count)
        axis = obstacle["axis"]
        if axis == "lateral":
            row[stage * NX + LATERAL] = 1.0
        elif axis == "effective-progress":
            row[stage * NX + PROGRESS] = 1.0
            row[stage * NX + LAG] = 1.0
        else:
            lateral_coefficient = float(obstacle["lateral_coefficient"])
            progress_coefficient = float(
                obstacle["effective_progress_coefficient"]
            )
            row[stage * NX + LATERAL] = lateral_coefficient
            row[stage * NX + PROGRESS] = progress_coefficient
            row[stage * NX + LAG] = progress_coefficient
        _bounded_rows(
            row,
            float(obstacle["lower"]),
            float(obstacle["upper"]),
            a_ub,
            b_ub,
            inequality_labels,
            f"dynamic_obstacle[{dynamic_count - 1}]",
        )

    bounds = [
        (float(lower), float(upper))
        for lower, upper in zip(
            np.concatenate((state_lower, input_lower)),
            np.concatenate((state_upper, input_upper)),
            strict=True,
        )
    ]
    def run(candidate_bounds: list[tuple[float | None, float | None]]):
        return linprog(
            np.zeros(variable_count),
            A_ub=np.asarray(a_ub) if a_ub else None,
            b_ub=np.asarray(b_ub) if b_ub else None,
            A_eq=np.asarray(a_eq),
            b_eq=np.asarray(b_eq),
            bounds=candidate_bounds,
            method="highs",
        )

    result = run(bounds)
    print(
        f"status={result.status} success={int(result.success)} "
        f"message={result.message!r} horizon={horizon} "
        f"variables={variable_count} equalities={len(a_eq)} "
        f"inequalities={len(a_ub)} dynamic_rows={dynamic_count}"
    )
    if not result.success:
        field_names = ("ey", "elag", "epsi", "v", "theta", "steer", "response")
        rescuing_relaxations: list[str] = []
        for variable, (lower, upper) in enumerate(bounds):
            if math.isfinite(lower):
                relaxed = list(bounds)
                relaxed[variable] = (None, upper)
                if run(relaxed).success:
                    if variable < state_count:
                        label = (
                            f"state[{variable // NX}]."
                            f"{field_names[variable % NX]}.lower"
                        )
                    else:
                        label = f"input[{(variable - state_count) // NU}].{(variable - state_count) % NU}.lower"
                    rescuing_relaxations.append(label)
            if math.isfinite(upper):
                relaxed = list(bounds)
                relaxed[variable] = (lower, None)
                if run(relaxed).success:
                    if variable < state_count:
                        label = (
                            f"state[{variable // NX}]."
                            f"{field_names[variable % NX]}.upper"
                        )
                    else:
                        label = f"input[{(variable - state_count) // NU}].{(variable - state_count) % NU}.upper"
                    rescuing_relaxations.append(label)
        print(
            "single_bound_relaxations=" +
            (",".join(rescuing_relaxations) if rescuing_relaxations else "none")
        )
        # A second, diagnostic-only LP keeps the recorded dynamics exact and
        # minimizes one common physical-unit slack across every inequality and
        # box.  It is not a production relaxation; active original violations
        # identify the conflicting constraint owners.
        elastic_variable_count = variable_count + 1
        elastic_a_ub: list[np.ndarray] = []
        elastic_b_ub: list[float] = []
        for row, upper_value in zip(a_ub, b_ub, strict=True):
            elastic_row = np.zeros(elastic_variable_count)
            elastic_row[:variable_count] = row
            elastic_row[-1] = -1.0
            elastic_a_ub.append(elastic_row)
            elastic_b_ub.append(upper_value)
        field_names = ("ey", "elag", "epsi", "v", "theta", "steer", "response")
        bound_labels: list[str] = []
        for variable, (lower, upper) in enumerate(bounds):
            if variable < state_count:
                base_label = (
                    f"state[{variable // NX}].{field_names[variable % NX]}"
                )
            else:
                base_label = (
                    f"input[{(variable - state_count) // NU}]."
                    f"{(variable - state_count) % NU}"
                )
            if math.isfinite(upper):
                elastic_row = np.zeros(elastic_variable_count)
                elastic_row[variable] = 1.0
                elastic_row[-1] = -1.0
                elastic_a_ub.append(elastic_row)
                elastic_b_ub.append(upper)
                bound_labels.append(f"{base_label}.upper")
            if math.isfinite(lower):
                elastic_row = np.zeros(elastic_variable_count)
                elastic_row[variable] = -1.0
                elastic_row[-1] = -1.0
                elastic_a_ub.append(elastic_row)
                elastic_b_ub.append(-lower)
                bound_labels.append(f"{base_label}.lower")
        elastic_a_eq = np.zeros((len(a_eq), elastic_variable_count))
        elastic_a_eq[:, :variable_count] = np.asarray(a_eq)
        elastic_cost = np.zeros(elastic_variable_count)
        elastic_cost[-1] = 1.0
        elastic = linprog(
            elastic_cost,
            A_ub=np.asarray(elastic_a_ub),
            b_ub=np.asarray(elastic_b_ub),
            A_eq=elastic_a_eq,
            b_eq=np.asarray(b_eq),
            bounds=[(None, None)] * variable_count + [(0.0, None)],
            method="highs",
        )
        print(
            f"elastic_success={int(elastic.success)} "
            f"minimum_common_slack={elastic.fun if elastic.success else math.nan:.9f}"
        )
        if elastic.success:
            candidate = np.asarray(elastic.x[:variable_count])
            violations: list[tuple[float, str]] = []
            for row, upper_value, label in zip(
                a_ub, b_ub, inequality_labels, strict=True
            ):
                violation = float(np.dot(row, candidate) - upper_value)
                if violation > 1e-9:
                    violations.append((violation, label))
            for variable, (lower, upper) in enumerate(bounds):
                if variable < state_count:
                    base_label = (
                        f"state[{variable // NX}].{field_names[variable % NX]}"
                    )
                else:
                    base_label = (
                        f"input[{(variable - state_count) // NU}."
                        f"{(variable - state_count) % NU}]"
                    )
                if math.isfinite(lower) and candidate[variable] < lower - 1e-9:
                    violations.append((lower - candidate[variable], f"{base_label}.lower"))
                if math.isfinite(upper) and candidate[variable] > upper + 1e-9:
                    violations.append((candidate[variable] - upper, f"{base_label}.upper"))
            for violation, label in sorted(violations, reverse=True)[:12]:
                print(f"elastic_violation={violation:.9f} constraint={label}")
        return 2
    solution = np.asarray(result.x)
    for stage in range(horizon + 1):
        state = solution[stage * NX : (stage + 1) * NX]
        print(
            f"stage={stage} ey={state[LATERAL]:.9f} "
            f"elag={state[LAG]:.9f} epsi={state[2]:.9f} "
            f"v={state[3]:.9f} theta={state[PROGRESS]:.9f} "
            f"steer={state[5]:.9f} response={state[6]:.9f}"
        )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    args = parser.parse_args()
    return solve(args.snapshot)


if __name__ == "__main__":
    raise SystemExit(main())
