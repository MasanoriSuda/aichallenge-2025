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


def rollout(document: dict, controls: np.ndarray) -> np.ndarray:
    source = document["source"]
    semantic = source["semantic_request"]
    assembly = document["assembly_request"]
    state = np.asarray(assembly["initial_state"], dtype=float)
    states = [state.copy()]
    wheelbase = float(semantic["wheelbase_m"])
    response_gain = float(semantic["yaw_response_gain"])
    response_tau = float(semantic["yaw_response_time_constant_sec"])
    minimum_denominator = float(semantic["minimum_frenet_denominator"])

    for stage, stage_input in enumerate(semantic["inputs"]):
        dt = float(stage_input["stage_dt_sec"])
        curvature = float(stage_input["path_curvature_radpm"])
        acceleration, steering_rate, progress_rate = controls[stage]
        ey, elag, epsi, velocity, progress, steering, response = state
        denominator = 1.0 - curvature * ey
        if denominator <= minimum_denominator:
            return np.full((len(controls) + 1, NX), np.nan)
        decay = math.exp(-dt / response_tau)
        response_from_steering = dt - response_tau * (1.0 - decay)
        response_from_rate = (
            0.5 * dt * dt
            - response_tau * dt
            + response_tau * response_tau * (1.0 - decay)
        )
        integrated_response = (
            math.tan(response) * dt
            + (1.0 / (math.cos(response) ** 2))
            * ((steering - response) * response_from_steering
               + steering_rate * response_from_rate)
        )
        state = np.asarray(
            [
                ey + dt * velocity * math.sin(epsi),
                elag + dt * (
                    velocity * math.cos(epsi) / denominator - progress_rate
                ),
                epsi
                + response_gain * velocity * integrated_response / wheelbase
                - dt * curvature * progress_rate,
                velocity + dt * acceleration,
                progress + dt * progress_rate,
                steering + dt * steering_rate,
                steering + (response - steering) * decay
                + steering_rate * response_from_steering,
            ],
            dtype=float,
        )
        states.append(state.copy())
    return np.asarray(states)


def constraint_values(document: dict, controls: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    assembly = document["assembly_request"]
    states = rollout(document, controls)
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
    print(
        f"snapshot={snapshot} attempts={attempts} "
        f"optimizer_converged={best.success} feasible={feasible} "
        f"minimum_slack={best_slack:.9g} iterations={best.nit} "
        f"message={best.message}"
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
