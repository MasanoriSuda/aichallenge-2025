#!/usr/bin/env python3
"""Audit exact seven-state feasibility of a frozen MPCC failure snapshot.

This tool deliberately ignores the recorded affine dynamics and rolls out the
same nonlinear seven-state Frenet/yaw-response model used by the production
physical adapter.  All recorded state/input boxes, progress-wall rows, swept
lateral rows and steering-prefix bounds remain hard constraints.  It therefore
separates an affine single-SQP failure from a genuinely infeasible temporal
maneuver without changing production solver settings.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from scipy.optimize import Bounds, differential_evolution, minimize


NX = 7
NU = 3
EY = 0
ELAG = 1
EPSI = 2
VELOCITY = 3
THETA = 4
STEERING = 5
RESPONSE_STEERING = 6
ACCELERATION = 0
STEERING_RATE = 1
VIRTUAL_PROGRESS_SPEED = 2


def transition(
    state: np.ndarray,
    control: np.ndarray,
    curvature: float,
    dt: float,
    wheelbase: float,
    yaw_gain: float,
    yaw_time_constant: float,
    minimum_denominator: float,
) -> np.ndarray | None:
    substeps = max(1, math.ceil(dt / 0.01))
    step = dt / substeps
    result = state.copy()
    for _ in range(substeps):
        steering = result[STEERING]
        response = result[RESPONSE_STEERING]
        steering_rate = control[STEERING_RATE]
        decay_mid = math.exp(-0.5 * step / yaw_time_constant)
        response_mid = (
            steering
            + (response - steering) * decay_mid
            + steering_rate
            * (0.5 * step - yaw_time_constant * (1.0 - decay_mid))
        )
        decay = math.exp(-step / yaw_time_constant)
        response_next = (
            steering
            + (response - steering) * decay
            + steering_rate * (step - yaw_time_constant * (1.0 - decay))
        )
        velocity_mid = result[VELOCITY] + 0.5 * control[ACCELERATION] * step
        heading_rate_mid = (
            yaw_gain * velocity_mid * math.tan(response_mid) / wheelbase
            - curvature * control[VIRTUAL_PROGRESS_SPEED]
        )
        heading_mid = result[EPSI] + 0.5 * heading_rate_mid * step
        lateral_rate_mid = velocity_mid * math.sin(heading_mid)
        lateral_mid = result[EY] + 0.5 * lateral_rate_mid * step
        denominator = 1.0 - curvature * lateral_mid
        if not math.isfinite(denominator) or denominator < minimum_denominator:
            return None
        physical_progress_rate = velocity_mid * math.cos(heading_mid) / denominator
        result[EY] += lateral_rate_mid * step
        result[ELAG] += (
            physical_progress_rate - control[VIRTUAL_PROGRESS_SPEED]
        ) * step
        result[EPSI] += heading_rate_mid * step
        result[VELOCITY] += control[ACCELERATION] * step
        result[THETA] += control[VIRTUAL_PROGRESS_SPEED] * step
        result[STEERING] += steering_rate * step
        result[RESPONSE_STEERING] = response_next
        if not np.all(np.isfinite(result)):
            return None
    return result


def load_snapshot(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    return document


class AuditProblem:
    def __init__(self, document: dict[str, Any]) -> None:
        source = document["source"]
        semantic = source["semantic_request"]
        assembly = document["assembly_request"]
        self.horizon = int(assembly["horizon_steps"])
        self.initial = np.asarray(assembly["initial_state"], dtype=float)
        self.state_lower = np.asarray(assembly["state_lower"], dtype=float).reshape(
            self.horizon + 1, NX
        )
        self.state_upper = np.asarray(assembly["state_upper"], dtype=float).reshape(
            self.horizon + 1, NX
        )
        self.input_lower = np.asarray(assembly["input_lower"], dtype=float).reshape(
            self.horizon, NU
        )
        self.input_upper = np.asarray(assembly["input_upper"], dtype=float).reshape(
            self.horizon, NU
        )
        self.input_reference = np.asarray(
            assembly["input_reference"], dtype=float
        ).reshape(self.horizon, NU)
        self.path_curvature = np.asarray(
            [stage["path_curvature_radpm"] for stage in semantic["inputs"]],
            dtype=float,
        )
        self.stage_dt = np.asarray(
            [stage["stage_dt_sec"] for stage in semantic["inputs"]], dtype=float
        )
        self.wheelbase = float(semantic["wheelbase_m"])
        self.yaw_gain = float(semantic["yaw_response_gain"])
        self.yaw_time_constant = float(
            semantic["yaw_response_time_constant_sec"]
        )
        self.minimum_denominator = float(semantic["minimum_frenet_denominator"])
        self.steering_prefix = assembly.get("steering_rate_prefix_bounds")
        self.progress_wall = assembly.get("progress_aligned_wall_constraints")
        self.swept_wall = assembly.get("swept_lateral_wall_constraints", [])
        self.dynamic = assembly.get("dynamic_obstacle_constraints", [])
        warm_start = document.get("warm_start", {})
        warm_primal = np.asarray(warm_start.get("primal", []), dtype=float)
        state_values = NX * (self.horizon + 1)
        if warm_primal.size == state_values + NU * self.horizon:
            self.seed = warm_primal[state_values:].reshape(self.horizon, NU)
        else:
            self.seed = self.input_reference.copy()

    def rollout(self, flat_controls: np.ndarray) -> np.ndarray | None:
        controls = flat_controls.reshape(self.horizon, NU)
        states = [self.initial]
        for stage in range(self.horizon):
            next_state = transition(
                states[-1],
                controls[stage],
                self.path_curvature[stage],
                self.stage_dt[stage],
                self.wheelbase,
                self.yaw_gain,
                self.yaw_time_constant,
                self.minimum_denominator,
            )
            if next_state is None:
                return None
            states.append(next_state)
        return np.asarray(states)

    @staticmethod
    def _append_interval_margins(
        margins: list[float], value: float, lower: float, upper: float
    ) -> None:
        if math.isfinite(lower):
            margins.append(value - lower)
        if math.isfinite(upper):
            margins.append(upper - value)

    def margins(self, flat_controls: np.ndarray) -> np.ndarray:
        states = self.rollout(flat_controls)
        if states is None:
            return np.asarray([-1.0e6])
        controls = flat_controls.reshape(self.horizon, NU)
        margins: list[float] = []
        for stage in range(1, self.horizon + 1):
            for element in range(NX):
                self._append_interval_margins(
                    margins,
                    states[stage, element],
                    self.state_lower[stage, element],
                    self.state_upper[stage, element],
                )
        if self.steering_prefix is not None:
            cumulative = 0.0
            for stage in range(self.horizon):
                cumulative += controls[stage, STEERING_RATE] * self.stage_dt[stage]
                self._append_interval_margins(
                    margins,
                    cumulative,
                    float(self.steering_prefix["minimum_cumulative_delta_rad"]),
                    float(self.steering_prefix["maximum_cumulative_delta_rad"]),
                )
        if self.progress_wall is not None:
            for stage in range(self.horizon):
                state = states[stage + 1]
                lower_value = (
                    state[EY]
                    - float(self.progress_wall["lower_slope"][stage])
                    * state[THETA]
                )
                upper_value = (
                    state[EY]
                    - float(self.progress_wall["upper_slope"][stage])
                    * state[THETA]
                )
                margins.append(
                    lower_value
                    - float(self.progress_wall["lower_intercept"][stage])
                )
                margins.append(
                    float(self.progress_wall["upper_intercept"][stage])
                    - upper_value
                )
        for wall in self.swept_wall:
            stage = int(wall["transition_stage"])
            ratio = float(wall["destination_ratio"])
            lateral = (
                (1.0 - ratio) * states[stage, EY]
                + ratio * states[stage + 1, EY]
            )
            self._append_interval_margins(
                margins,
                lateral,
                float(wall["lower_m"]),
                float(wall["upper_m"]),
            )
        for obstacle in self.dynamic:
            stage = int(obstacle["state_stage"])
            axis = obstacle["axis"]
            if axis == "lateral":
                value = states[stage, EY]
            elif axis == "effective-progress":
                value = states[stage, THETA] + states[stage, ELAG]
            else:
                value = (
                    float(obstacle["lateral_coefficient"]) * states[stage, EY]
                    + float(obstacle["effective_progress_coefficient"])
                    * (states[stage, THETA] + states[stage, ELAG])
                )
            self._append_interval_margins(
                margins,
                value,
                float(obstacle["lower"]),
                float(obstacle["upper"]),
            )
        return np.asarray(margins)

    def penalty(self, flat_controls: np.ndarray) -> float:
        margins = self.margins(flat_controls)
        violations = np.minimum(margins, 0.0)
        return float(np.dot(violations, violations))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--global-search", action="store_true")
    args = parser.parse_args()
    problem = AuditProblem(load_snapshot(args.snapshot))
    lower = problem.input_lower.reshape(-1)
    upper = problem.input_upper.reshape(-1)
    seeds = [problem.seed.reshape(-1), problem.input_reference.reshape(-1)]
    if args.global_search:
        global_result = differential_evolution(
            problem.penalty,
            list(zip(lower, upper)),
            seed=17,
            popsize=20,
            maxiter=600,
            polish=False,
            workers=1,
            updating="immediate",
        )
        seeds.insert(0, global_result.x)
        print(
            f"global_success={int(global_result.success)} "
            f"global_penalty={global_result.fun:.12g}"
        )
    best = None
    for seed_index, seed in enumerate(seeds):
        result = minimize(
            problem.penalty,
            np.clip(seed, lower, upper),
            method="SLSQP",
            bounds=Bounds(lower, upper),
            options={"maxiter": 4000, "ftol": 1.0e-15, "disp": False},
        )
        margins = problem.margins(result.x)
        minimum_margin = float(np.min(margins))
        penalty = problem.penalty(result.x)
        print(
            f"seed={seed_index} optimizer_success={int(result.success)} "
            f"penalty={penalty:.12g} minimum_margin={minimum_margin:.12g} "
            f"iterations={result.nit} message={result.message!r}"
        )
        if best is None or penalty < best[0]:
            best = (penalty, minimum_margin, result.x)
    assert best is not None
    states = problem.rollout(best[2])
    print(
        f"nonlinear_feasible={int(best[0] <= 1.0e-14 and best[1] >= -1.0e-7)} "
        f"best_penalty={best[0]:.12g} best_minimum_margin={best[1]:.12g}"
    )
    if states is not None:
        for stage, state in enumerate(states):
            print(
                f"state={stage} ey={state[EY]:.9f} elag={state[ELAG]:.9f} "
                f"epsi={state[EPSI]:.9f} v={state[VELOCITY]:.9f} "
                f"theta={state[THETA]:.9f} steering={state[STEERING]:.9f} "
                f"response={state[RESPONSE_STEERING]:.9f}"
            )
    return 0 if best[0] <= 1.0e-14 and best[1] >= -1.0e-7 else 1


if __name__ == "__main__":
    raise SystemExit(main())
