#!/usr/bin/env python3
"""Search a frozen MPCC snapshot with exact nonlinear dynamics.

This is an observation-only architecture oracle.  It deliberately does not
enforce the affine lag/heading wall buckets which are under audit.  It keeps
the recorded input limits, physical state limits, progress schedule, steering
prefix, progress-aligned/swept wall rows and dynamic-obstacle rows.  The output
is a complete primal whose states are rebuilt from the exact nonlinear
seven-state rollout; the C++ comparison tool remains the authority for the
unchanged exact trajectory, wall, obstacle and successor proofs.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from scipy.optimize import Bounds, minimize


NX = 7
NU = 3
EY, ELAG, EPSI, VELOCITY, PROGRESS, STEERING, RESPONSE = range(NX)
ACCELERATION, STEERING_RATE, PROGRESS_RATE = range(NU)
MAXIMUM_ROLLOUT_STEP_SEC = 0.01


def response_after_ramp(
    command: float,
    response: float,
    steering_rate: float,
    elapsed_sec: float,
    time_constant_sec: float,
) -> float:
    decay = math.exp(-elapsed_sec / time_constant_sec)
    return (
        command
        + (response - command) * decay
        + steering_rate
        * (elapsed_sec - time_constant_sec * (1.0 - decay))
    )


def advance(
    state: np.ndarray,
    control: np.ndarray,
    curvature: float,
    step_sec: float,
    wheelbase_m: float,
    yaw_gain: float,
    yaw_time_constant_sec: float,
    minimum_denominator: float,
) -> np.ndarray | None:
    acceleration, steering_rate, progress_rate = control
    ey, elag, epsi, velocity, progress, steering, response = state
    response_mid = response_after_ramp(
        steering,
        response,
        steering_rate,
        0.5 * step_sec,
        yaw_time_constant_sec,
    )
    response_next = response_after_ramp(
        steering,
        response,
        steering_rate,
        step_sec,
        yaw_time_constant_sec,
    )
    velocity_mid = velocity + 0.5 * acceleration * step_sec
    heading_rate_mid = (
        yaw_gain * velocity_mid * math.tan(response_mid) / wheelbase_m
        - curvature * progress_rate
    )
    heading_mid = epsi + 0.5 * heading_rate_mid * step_sec
    lateral_rate_mid = velocity_mid * math.sin(heading_mid)
    lateral_mid = ey + 0.5 * lateral_rate_mid * step_sec
    denominator = 1.0 - curvature * lateral_mid
    if not math.isfinite(denominator) or denominator < minimum_denominator:
        return None
    physical_progress_rate = velocity_mid * math.cos(heading_mid) / denominator
    result = np.asarray(
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
    return result if np.all(np.isfinite(result)) else None


class PhysicalProblem:
    def __init__(self, document: dict[str, Any], seed_primal: np.ndarray) -> None:
        source = document["source"]
        semantic = source["semantic_request"]
        assembly = document["assembly_request"]
        self.horizon = int(assembly["horizon_steps"])
        self.initial = np.asarray(assembly["initial_state"], dtype=float)
        self.state_lower = np.asarray(
            assembly["state_lower"], dtype=float
        ).reshape(self.horizon + 1, NX)
        self.state_upper = np.asarray(
            assembly["state_upper"], dtype=float
        ).reshape(self.horizon + 1, NX)
        self.input_lower = np.asarray(
            assembly["input_lower"], dtype=float
        ).reshape(self.horizon, NU)
        self.input_upper = np.asarray(
            assembly["input_upper"], dtype=float
        ).reshape(self.horizon, NU)
        variable_count = NX * (self.horizon + 1) + NU * self.horizon
        if seed_primal.size != variable_count:
            raise ValueError(
                f"seed primal has {seed_primal.size} values, expected {variable_count}"
            )
        self.seed_controls = seed_primal[NX * (self.horizon + 1) :].reshape(
            self.horizon, NU
        )
        self.input_reference = np.asarray(
            assembly["input_reference"], dtype=float
        ).reshape(self.horizon, NU)
        self.stage_inputs = semantic["inputs"]
        self.wheelbase_m = float(semantic["wheelbase_m"])
        self.yaw_gain = float(semantic["yaw_response_gain"])
        self.yaw_time_constant_sec = float(
            semantic["yaw_response_time_constant_sec"]
        )
        self.minimum_denominator = float(semantic["minimum_frenet_denominator"])
        self.steering_prefix = assembly.get("steering_rate_prefix_bounds")
        self.progress_wall = assembly.get("progress_aligned_wall_constraints")
        self.swept_wall = assembly.get("swept_lateral_wall_constraints", [])
        self.dynamic = assembly.get("dynamic_obstacle_constraints", [])

    def rollout(
        self, flat_controls: np.ndarray
    ) -> tuple[np.ndarray | None, list[tuple[int, float, np.ndarray]]]:
        controls = flat_controls.reshape(self.horizon, NU)
        state = self.initial.copy()
        states = [state.copy()]
        dense: list[tuple[int, float, np.ndarray]] = []
        for stage, semantic in enumerate(self.stage_inputs):
            duration = float(semantic["stage_dt_sec"])
            curvature = float(semantic["path_curvature_radpm"])
            substeps = max(1, math.ceil(duration / MAXIMUM_ROLLOUT_STEP_SEC))
            step_sec = duration / substeps
            for substep in range(substeps):
                next_state = advance(
                    state,
                    controls[stage],
                    curvature,
                    step_sec,
                    self.wheelbase_m,
                    self.yaw_gain,
                    self.yaw_time_constant_sec,
                    self.minimum_denominator,
                )
                if next_state is None:
                    return None, []
                state = next_state
                dense.append((stage, (substep + 1) / substeps, state.copy()))
            states.append(state.copy())
        return np.asarray(states), dense

    @staticmethod
    def append_interval(
        margins: list[float], value: float, lower: float, upper: float
    ) -> None:
        if math.isfinite(lower):
            margins.append(value - lower)
        if math.isfinite(upper):
            margins.append(upper - value)

    def margins(self, flat_controls: np.ndarray) -> np.ndarray:
        states, dense = self.rollout(flat_controls)
        if states is None:
            return np.asarray([-1.0e6])
        controls = flat_controls.reshape(self.horizon, NU)
        margins: list[float] = []

        # Lag and heading boxes are the artificial post-hoc wall buckets under
        # audit.  Physical lateral position, speed, progress and actuator
        # states remain constrained.
        for stage in range(1, self.horizon + 1):
            for element in (EY, VELOCITY, PROGRESS, STEERING, RESPONSE):
                self.append_interval(
                    margins,
                    states[stage, element],
                    self.state_lower[stage, element],
                    self.state_upper[stage, element],
                )

        # Match the production exact adapter: wall bounds apply throughout
        # every stage, not only at the QP knot states.
        for stage, fraction, state in dense:
            lower = (
                (1.0 - fraction) * self.state_lower[stage, EY]
                + fraction * self.state_lower[stage + 1, EY]
            )
            upper = (
                (1.0 - fraction) * self.state_upper[stage, EY]
                + fraction * self.state_upper[stage + 1, EY]
            )
            self.append_interval(margins, state[EY], lower, upper)

        if self.steering_prefix is not None:
            cumulative = 0.0
            for stage, semantic in enumerate(self.stage_inputs):
                cumulative += controls[stage, STEERING_RATE] * float(
                    semantic["stage_dt_sec"]
                )
                self.append_interval(
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
                    * state[PROGRESS]
                )
                upper_value = (
                    state[EY]
                    - float(self.progress_wall["upper_slope"][stage])
                    * state[PROGRESS]
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
            self.append_interval(
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
                value = states[stage, PROGRESS] + states[stage, ELAG]
            else:
                value = (
                    float(obstacle["lateral_coefficient"]) * states[stage, EY]
                    + float(obstacle["effective_progress_coefficient"])
                    * (states[stage, PROGRESS] + states[stage, ELAG])
                )
            self.append_interval(
                margins,
                value,
                float(obstacle["lower"]),
                float(obstacle["upper"]),
            )
        return np.asarray(margins)

    def penalty(self, flat_controls: np.ndarray) -> float:
        margins = self.margins(flat_controls)
        violation = np.minimum(margins, 0.0)
        scale = np.maximum(
            1.0, (self.input_upper - self.input_lower).reshape(-1)
        )
        control_delta = (flat_controls - self.seed_controls.reshape(-1)) / scale
        return float(np.dot(violation, violation) + 1.0e-10 * np.dot(control_delta, control_delta))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("seed_primal", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maxiter", type=int, default=800)
    args = parser.parse_args()

    document = yaml.safe_load(args.snapshot.read_text(encoding="utf-8"))
    seed_primal = np.loadtxt(args.seed_primal, dtype=float).reshape(-1)
    problem = PhysicalProblem(document, seed_primal)
    lower = problem.input_lower.reshape(-1)
    upper = problem.input_upper.reshape(-1)
    initial = np.clip(problem.seed_controls.reshape(-1), lower, upper)
    before = problem.margins(initial)
    result = minimize(
        problem.penalty,
        initial,
        method="L-BFGS-B",
        bounds=Bounds(lower, upper),
        options={"maxiter": args.maxiter, "ftol": 1.0e-15, "gtol": 1.0e-10},
    )
    after = problem.margins(result.x)
    states, _ = problem.rollout(result.x)
    if states is None:
        raise RuntimeError("optimized controls do not produce a finite rollout")
    primal = np.r_[states.reshape(-1), result.x]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.output, primal)
    minimum_before = float(np.min(before))
    minimum_after = float(np.min(after))
    feasible = minimum_after >= -1.0e-6
    print(
        f"optimizer_success={int(result.success)} feasible={int(feasible)} "
        f"minimum_before={minimum_before:.12g} "
        f"minimum_after={minimum_after:.12g} iterations={result.nit} "
        f"objective={result.fun:.12g} message={result.message!r} "
        f"output={args.output}"
    )
    return 0 if feasible else 1


if __name__ == "__main__":
    raise SystemExit(main())
