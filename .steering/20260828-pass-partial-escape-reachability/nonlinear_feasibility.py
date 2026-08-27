#!/usr/bin/env python3
"""Bounded exact-dynamics feasibility probe for a frozen MPCC snapshot.

This is audit-only tooling.  It changes neither the recorded constraints nor
production authority.  Failure to find a solution is inconclusive.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import minimize


NX = 7
NU = 3
EY, ELAG, EPSI, VELOCITY, PROGRESS, STEERING, RESPONSE = range(NX)
ACCELERATION, STEERING_RATE, PROGRESS_RATE = range(NU)


MAXIMUM_NONLINEAR_ROLLOUT_STEP_SEC = 0.01


def _response_steering_after_ramp(
    initial_command: float,
    initial_response: float,
    steering_rate: float,
    elapsed: float,
    time_constant: float,
) -> float:
    decay = math.exp(-elapsed / time_constant)
    return (
        initial_command
        + (initial_response - initial_command) * decay
        + steering_rate * (elapsed - time_constant * (1.0 - decay))
    )


def _advance_production_nonlinear_state(
    state: np.ndarray,
    control: np.ndarray,
    curvature: float,
    step_sec: float,
    wheelbase: float,
    response_gain: float,
    response_tau: float,
    minimum_denominator: float,
) -> np.ndarray | None:
    acceleration, steering_rate, progress_rate = control
    ey, elag, epsi, velocity, progress, steering, response = state
    response_mid = _response_steering_after_ramp(
        steering, response, steering_rate, 0.5 * step_sec, response_tau
    )
    response_next = _response_steering_after_ramp(
        steering, response, steering_rate, step_sec, response_tau
    )
    velocity_mid = velocity + 0.5 * acceleration * step_sec
    heading_rate_mid = (
        response_gain * velocity_mid * math.tan(response_mid) / wheelbase
        - curvature * progress_rate
    )
    heading_mid = epsi + 0.5 * heading_rate_mid * step_sec
    lateral_rate_mid = velocity_mid * math.sin(heading_mid)
    lateral_mid = ey + 0.5 * lateral_rate_mid * step_sec
    denominator = 1.0 - curvature * lateral_mid
    if denominator < minimum_denominator:
        return None
    physical_progress_rate = velocity_mid * math.cos(heading_mid) / denominator
    next_state = np.asarray(
        [
            ey + lateral_rate_mid * step_sec,
            elag + (physical_progress_rate - progress_rate) * step_sec,
            epsi + heading_rate_mid * step_sec,
            velocity + acceleration * step_sec,
            progress + progress_rate * step_sec,
            steering + steering_rate * step_sec,
            response_next,
        ],
        dtype=float,
    )
    return next_state if np.all(np.isfinite(next_state)) else None


def rollout(
    document: dict, controls: np.ndarray
) -> tuple[np.ndarray, list[tuple[int, float, np.ndarray]]]:
    source = document["source"]
    semantic = source["semantic_request"]
    assembly = document["assembly_request"]
    state = np.asarray(assembly["initial_state"], dtype=float)
    states = [state.copy()]
    wheelbase = float(semantic["wheelbase_m"])
    response_gain = float(semantic["yaw_response_gain"])
    response_tau = float(semantic["yaw_response_time_constant_sec"])
    minimum_denominator = float(semantic["minimum_frenet_denominator"])

    dense_states: list[tuple[int, float, np.ndarray]] = []
    for stage, stage_input in enumerate(semantic["inputs"]):
        dt = float(stage_input["stage_dt_sec"])
        curvature = float(stage_input["path_curvature_radpm"])
        substep_count = max(1, math.ceil(dt / MAXIMUM_NONLINEAR_ROLLOUT_STEP_SEC))
        step_sec = dt / substep_count
        for substep in range(substep_count):
            next_state = _advance_production_nonlinear_state(
                state,
                controls[stage],
                curvature,
                step_sec,
                wheelbase,
                response_gain,
                response_tau,
                minimum_denominator,
            )
            if next_state is None:
                invalid = np.full((len(controls) + 1, NX), np.nan)
                return invalid, []
            state = next_state
            dense_states.append((stage, (substep + 1) / substep_count, state.copy()))
        states.append(state.copy())
    return np.asarray(states), dense_states


def constraint_values(document: dict, controls: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    assembly = document["assembly_request"]
    states, dense_states = rollout(document, controls)
    if not np.all(np.isfinite(states)):
        return np.asarray([np.nan]), np.asarray([0.0]), np.asarray([0.0])

    values: list[float] = []
    lower: list[float] = []
    upper: list[float] = []

    state_lower = np.asarray(assembly["state_lower"], dtype=float).reshape((-1, NX))
    state_upper = np.asarray(assembly["state_upper"], dtype=float).reshape((-1, NX))
    for stage in range(states.shape[0]):
        values.extend(states[stage])
        lower.extend(state_lower[stage])
        upper.extend(state_upper[stage])

    # Production does not certify only the QP knot states. It replays the
    # command sequence with <=10 ms nonlinear substeps and checks every sample
    # against linearly interpolated lateral bounds. Keep the offline D probe
    # identical to that proof; otherwise endpoint-only feasibility can be
    # misclassified as a single-SQP limitation.
    physical_tolerance = float(
        document["production_outcome"]["telemetry"]["physical_global_tolerance"]
    )
    for stage, fraction, dense_state in dense_states:
        interpolate = 1.0 - fraction
        dense_lower = (
            interpolate * state_lower[stage, EY]
            + fraction * state_lower[stage + 1, EY]
        )
        dense_upper = (
            interpolate * state_upper[stage, EY]
            + fraction * state_upper[stage + 1, EY]
        )
        values.append(dense_state[EY])
        lower.append(dense_lower - physical_tolerance)
        upper.append(dense_upper + physical_tolerance)

    prefix = assembly.get("steering_rate_prefix_bounds")
    if prefix:
        cumulative = 0.0
        for stage, item in enumerate(document["source"]["semantic_request"]["inputs"]):
            cumulative += controls[stage, STEERING_RATE] * float(item["stage_dt_sec"])
            values.append(cumulative)
            lower.append(float(prefix["minimum_cumulative_delta_rad"]))
            upper.append(float(prefix["maximum_cumulative_delta_rad"]))

    wall = assembly.get("progress_aligned_wall_constraints")
    if wall:
        for stage in range(len(controls)):
            state = states[stage + 1]
            values.extend(
                [
                    state[EY] - float(wall["lower_slope"][stage]) * state[PROGRESS],
                    state[EY] - float(wall["upper_slope"][stage]) * state[PROGRESS],
                ]
            )
            lower.extend([float(wall["lower_intercept"][stage]), -math.inf])
            upper.extend([math.inf, float(wall["upper_intercept"][stage])])

    for swept in assembly.get("swept_lateral_wall_constraints", []):
        stage = int(swept["transition_stage"])
        ratio = float(swept["destination_ratio"])
        values.append((1.0 - ratio) * states[stage, EY] + ratio * states[stage + 1, EY])
        lower.append(float(swept["lower_m"]))
        upper.append(float(swept["upper_m"]))

    for obstacle in assembly.get("dynamic_obstacle_constraints", []):
        state = states[int(obstacle["state_stage"])]
        if obstacle["axis"] == "lateral":
            values.append(state[EY])
        else:
            values.append(state[PROGRESS] + state[ELAG])
        lower.append(float(obstacle["lower"]))
        upper.append(float(obstacle["upper"]))

    return np.asarray(values), np.asarray(lower), np.asarray(upper)


def residuals(document: dict, flat_controls: np.ndarray) -> np.ndarray:
    controls = flat_controls.reshape((-1, NU))
    values, lower, upper = constraint_values(document, controls)
    if not np.all(np.isfinite(values)):
        return np.asarray([-1.0e6])
    parts = []
    finite_lower = np.isfinite(lower)
    finite_upper = np.isfinite(upper)
    parts.append(values[finite_lower] - lower[finite_lower])
    parts.append(upper[finite_upper] - values[finite_upper])
    return np.concatenate(parts)


def dense_physical_diagnostic(
    document: dict, controls: np.ndarray
) -> tuple[float, int, float, float, float, float]:
    assembly = document["assembly_request"]
    states, dense_states = rollout(document, controls)
    if not np.all(np.isfinite(states)):
        return -math.inf, -1, math.nan, math.nan, math.nan, math.nan
    state_lower = np.asarray(assembly["state_lower"], dtype=float).reshape((-1, NX))
    state_upper = np.asarray(assembly["state_upper"], dtype=float).reshape((-1, NX))
    tolerance = float(
        document["production_outcome"]["telemetry"]["physical_global_tolerance"]
    )
    best = (math.inf, -1, math.nan, math.nan, math.nan, math.nan)
    for stage, fraction, state in dense_states:
        interpolate = 1.0 - fraction
        lower = (
            interpolate * state_lower[stage, EY]
            + fraction * state_lower[stage + 1, EY]
        )
        upper = (
            interpolate * state_upper[stage, EY]
            + fraction * state_upper[stage + 1, EY]
        )
        slack = min(state[EY] - lower + tolerance, upper - state[EY] + tolerance)
        if slack < best[0]:
            best = (slack, stage, fraction, state[EY], lower, upper)
    return best


def solve(snapshot: Path, attempts: int) -> int:
    document = yaml.safe_load(snapshot.read_text(encoding="utf-8"))
    assembly = document["assembly_request"]
    horizon = int(assembly["horizon_steps"])
    reference = np.asarray(assembly["input_reference"], dtype=float).reshape((horizon, NU))
    lower = np.asarray(assembly["input_lower"], dtype=float).reshape((horizon, NU))
    upper = np.asarray(assembly["input_upper"], dtype=float).reshape((horizon, NU))
    reference = np.clip(reference, lower, upper)
    control_bounds = list(zip(lower.ravel(), upper.ravel()))
    # Maximize the worst hard-constraint slack explicitly.  A feasibility-only
    # SLSQP objective can stop at a locally stationary but still violated
    # point and would make a negative audit result even less informative.
    bounds = control_bounds + [(-10.0, 1.0)]
    rng = np.random.default_rng(20260828)
    best = None
    best_slack = -math.inf

    def objective(decision: np.ndarray) -> float:
        flat_controls = decision[:-1]
        worst_slack = decision[-1]
        normalized = (flat_controls - reference.ravel()) / np.maximum(1.0, upper.ravel() - lower.ravel())
        return float(-worst_slack + 1.0e-6 * np.dot(normalized, normalized))

    def slack_constraints(decision: np.ndarray) -> np.ndarray:
        return residuals(document, decision[:-1]) - decision[-1]

    for attempt in range(attempts):
        if attempt == 0:
            initial_controls = reference.ravel()
        else:
            initial_controls = rng.uniform(lower, upper).ravel()
        initial_residuals = residuals(document, initial_controls)
        initial = np.r_[initial_controls, max(-10.0, min(0.0, float(np.min(initial_residuals))))]
        result = minimize(
            objective,
            initial,
            method="SLSQP",
            bounds=bounds,
            constraints=[{"type": "ineq", "fun": slack_constraints}],
            options={"maxiter": 3000, "ftol": 1.0e-10, "disp": False},
        )
        current_residuals = residuals(document, result.x[:-1])
        minimum_slack = float(np.min(current_residuals))
        if minimum_slack > best_slack:
            best = result
            best_slack = minimum_slack
        if result.success and minimum_slack >= -1.0e-6:
            break

    assert best is not None
    feasible = best.success and best_slack >= -1.0e-6
    optimized_controls = best.x[:-1].reshape((horizon, NU))
    optimized_dense = dense_physical_diagnostic(document, optimized_controls)
    state_values = NX * (horizon + 1)
    production_primal = np.asarray(
        document["production_outcome"]["result"]["primal"], dtype=float
    )
    production_controls = production_primal[state_values:].reshape((horizon, NU))
    production_dense = dense_physical_diagnostic(document, production_controls)
    print(
        f"snapshot={snapshot} attempts={attempts} "
        f"optimizer_converged={best.success} feasible={feasible} "
        f"minimum_slack={best_slack:.9g} iterations={best.nit} "
        f"message={best.message}\n"
        f"production_dense_slack={production_dense[0]:.9g} "
        f"stage={production_dense[1]} fraction={production_dense[2]:.6g} "
        f"lateral={production_dense[3]:.9g} "
        f"bounds=[{production_dense[4]:.9g},{production_dense[5]:.9g}]\n"
        f"optimized_dense_slack={optimized_dense[0]:.9g} "
        f"stage={optimized_dense[1]} fraction={optimized_dense[2]:.6g} "
        f"lateral={optimized_dense[3]:.9g} "
        f"bounds=[{optimized_dense[4]:.9g},{optimized_dense[5]:.9g}] "
        f"control_l2_delta={np.linalg.norm(optimized_controls - production_controls):.9g}"
    )
    return 0 if feasible else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--attempts", type=int, default=32)
    args = parser.parse_args()
    if args.attempts <= 0:
        parser.error("--attempts must be positive")
    return solve(args.snapshot, args.attempts)


if __name__ == "__main__":
    raise SystemExit(main())
