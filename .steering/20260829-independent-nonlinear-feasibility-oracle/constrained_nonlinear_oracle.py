#!/usr/bin/env python3
"""Deterministic offline nonlinear feasibility oracle for frozen MPCC data.

The script has no production authority.  Its output must pass the existing C++
external-primal exact proof before it can be treated as architecture evidence.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
from pathlib import Path
from typing import Any

import casadi as ca
import numpy as np
import yaml


NX = 7
NU = 3
EY, ELAG, EPSI, VELOCITY, PROGRESS, STEERING, RESPONSE = range(NX)
ACCELERATION, STEERING_RATE, PROGRESS_RATE = range(NU)
MAXIMUM_ROLLOUT_STEP_SEC = 0.01
FEASIBILITY_TOLERANCE = 1.0e-6


def load_numeric_oracle():
    source = Path(__file__).parents[1] / (
        "20260829-wall-bucket-physical-oracle/nonlinear_physical_oracle.py"
    )
    spec = importlib.util.spec_from_file_location("numeric_oracle", source)
    if spec is None or spec.loader is None:
        raise RuntimeError("numeric nonlinear oracle unavailable")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def response_after_ramp(
    command: Any,
    response: Any,
    steering_rate: Any,
    elapsed_sec: float,
    time_constant_sec: float,
):
    decay = math.exp(-elapsed_sec / time_constant_sec)
    return (
        command
        + (response - command) * decay
        + steering_rate
        * (elapsed_sec - time_constant_sec * (1.0 - decay))
    )


class ConstrainedOracle:
    def __init__(self, physical: Any) -> None:
        self.physical = physical
        self.controls = ca.MX.sym("controls", physical.horizon * NU)
        self.slack = ca.MX.sym("maximum_physical_violation")
        self.states: list[Any] = [ca.DM(physical.initial)]
        self.dense: list[tuple[int, float, Any]] = []
        state = self.states[0]
        for stage, semantic in enumerate(physical.stage_inputs):
            duration = float(semantic["stage_dt_sec"])
            curvature = float(semantic["path_curvature_radpm"])
            substeps = max(1, math.ceil(duration / MAXIMUM_ROLLOUT_STEP_SEC))
            step_sec = duration / substeps
            control = self.controls[stage * NU : (stage + 1) * NU]
            for substep in range(substeps):
                state = self.advance(state, control, curvature, step_sec)
                self.dense.append(
                    (stage, (substep + 1) / substeps, state)
                )
            self.states.append(state)

        self.margins: list[Any] = []
        self.labels: list[str] = []
        self.build_physical_margins()
        margin_vector = ca.vertcat(*self.margins)
        self.margin_function = ca.Function(
            "retained_physical_margins", [self.controls], [margin_vector]
        )
        scale = np.maximum(
            1.0,
            (physical.input_upper - physical.input_lower).reshape(-1),
        )
        seed = physical.seed_controls.reshape(-1)
        normalized_delta = (self.controls - seed) / scale
        objective = self.slack + 1.0e-8 * ca.sumsqr(normalized_delta)
        variables = ca.vertcat(self.controls, self.slack)
        self.solver = ca.nlpsol(
            "physical_feasibility",
            "ipopt",
            {"x": variables, "f": objective, "g": margin_vector + self.slack},
            {
                "print_time": False,
                "error_on_fail": False,
                "ipopt.sb": "yes",
                "ipopt.print_level": 0,
                "ipopt.max_iter": 2000,
                "ipopt.tol": 1.0e-9,
                "ipopt.acceptable_tol": 1.0e-7,
            },
        )
        self.lower = np.r_[physical.input_lower.reshape(-1), 0.0]
        self.upper = np.r_[physical.input_upper.reshape(-1), 100.0]
        self.constraint_lower = np.zeros(len(self.margins))
        self.constraint_upper = np.full(len(self.margins), np.inf)

    def advance(self, state: Any, control: Any, curvature: float, step_sec: float):
        acceleration = control[ACCELERATION]
        steering_rate = control[STEERING_RATE]
        progress_rate = control[PROGRESS_RATE]
        ey = state[EY]
        elag = state[ELAG]
        epsi = state[EPSI]
        velocity = state[VELOCITY]
        progress = state[PROGRESS]
        steering = state[STEERING]
        response = state[RESPONSE]
        response_mid = response_after_ramp(
            steering,
            response,
            steering_rate,
            0.5 * step_sec,
            self.physical.yaw_time_constant_sec,
        )
        response_next = response_after_ramp(
            steering,
            response,
            steering_rate,
            step_sec,
            self.physical.yaw_time_constant_sec,
        )
        velocity_mid = velocity + 0.5 * acceleration * step_sec
        heading_rate_mid = (
            self.physical.yaw_gain
            * velocity_mid
            * ca.tan(response_mid)
            / self.physical.wheelbase_m
            - curvature * progress_rate
        )
        heading_mid = epsi + 0.5 * heading_rate_mid * step_sec
        lateral_rate_mid = velocity_mid * ca.sin(heading_mid)
        lateral_mid = ey + 0.5 * lateral_rate_mid * step_sec
        denominator = 1.0 - curvature * lateral_mid
        physical_progress_rate = velocity_mid * ca.cos(heading_mid) / denominator
        return ca.vertcat(
            ey + lateral_rate_mid * step_sec,
            elag + (physical_progress_rate - progress_rate) * step_sec,
            epsi + heading_rate_mid * step_sec,
            velocity + acceleration * step_sec,
            progress + progress_rate * step_sec,
            steering + steering_rate * step_sec,
            response_next,
        )

    def append_interval(
        self, value: Any, lower: float, upper: float, label: str
    ) -> None:
        if math.isfinite(lower):
            self.margins.append(value - lower)
            self.labels.append(f"{label}:lower")
        if math.isfinite(upper):
            self.margins.append(upper - value)
            self.labels.append(f"{label}:upper")

    def build_physical_margins(self) -> None:
        physical = self.physical
        for stage in range(1, physical.horizon + 1):
            for element, name in (
                (EY, "ey"),
                (VELOCITY, "velocity"),
                (PROGRESS, "progress"),
                (STEERING, "steering"),
                (RESPONSE, "response"),
            ):
                self.append_interval(
                    self.states[stage][element],
                    float(physical.state_lower[stage, element]),
                    float(physical.state_upper[stage, element]),
                    f"state[{stage}].{name}",
                )

        for stage, fraction, state in self.dense:
            lower = (
                (1.0 - fraction) * physical.state_lower[stage, EY]
                + fraction * physical.state_lower[stage + 1, EY]
            )
            upper = (
                (1.0 - fraction) * physical.state_upper[stage, EY]
                + fraction * physical.state_upper[stage + 1, EY]
            )
            self.append_interval(
                state[EY],
                float(lower),
                float(upper),
                f"dense-wall[{stage},{fraction:.6f}]",
            )

        if physical.steering_prefix is not None:
            cumulative: Any = 0.0
            for stage, semantic in enumerate(physical.stage_inputs):
                cumulative += self.controls[stage * NU + STEERING_RATE] * float(
                    semantic["stage_dt_sec"]
                )
                self.append_interval(
                    cumulative,
                    float(
                        physical.steering_prefix[
                            "minimum_cumulative_delta_rad"
                        ]
                    ),
                    float(
                        physical.steering_prefix[
                            "maximum_cumulative_delta_rad"
                        ]
                    ),
                    f"steering-prefix[{stage}]",
                )

        if physical.progress_wall is not None:
            for stage in range(physical.horizon):
                state = self.states[stage + 1]
                lower_value = (
                    state[EY]
                    - float(physical.progress_wall["lower_slope"][stage])
                    * state[PROGRESS]
                )
                upper_value = (
                    state[EY]
                    - float(physical.progress_wall["upper_slope"][stage])
                    * state[PROGRESS]
                )
                lower_intercept = float(
                    physical.progress_wall["lower_intercept"][stage]
                )
                upper_intercept = float(
                    physical.progress_wall["upper_intercept"][stage]
                )
                if math.isfinite(lower_intercept):
                    self.margins.append(lower_value - lower_intercept)
                    self.labels.append(f"progress-wall[{stage}]:lower")
                if math.isfinite(upper_intercept):
                    self.margins.append(upper_intercept - upper_value)
                    self.labels.append(f"progress-wall[{stage}]:upper")

        for index, wall in enumerate(physical.swept_wall):
            stage = int(wall["transition_stage"])
            ratio = float(wall["destination_ratio"])
            lateral = (
                (1.0 - ratio) * self.states[stage][EY]
                + ratio * self.states[stage + 1][EY]
            )
            self.append_interval(
                lateral,
                float(wall["lower_m"]),
                float(wall["upper_m"]),
                f"swept-wall[{index}]",
            )

        for index, obstacle in enumerate(physical.dynamic):
            stage = int(obstacle["state_stage"])
            axis = obstacle["axis"]
            if axis == "lateral":
                value = self.states[stage][EY]
            elif axis == "effective-progress":
                value = self.states[stage][PROGRESS] + self.states[stage][ELAG]
            else:
                value = (
                    float(obstacle["lateral_coefficient"])
                    * self.states[stage][EY]
                    + float(obstacle["effective_progress_coefficient"])
                    * (
                        self.states[stage][PROGRESS]
                        + self.states[stage][ELAG]
                    )
                )
            self.append_interval(
                value,
                float(obstacle["lower"]),
                float(obstacle["upper"]),
                f"dynamic[{index}].{axis}",
            )

    def solve(self, start: np.ndarray) -> tuple[np.ndarray, float, dict[str, Any]]:
        initial_controls = np.clip(
            start,
            self.physical.input_lower.reshape(-1),
            self.physical.input_upper.reshape(-1),
        )
        initial_margins = self.evaluate_margins(initial_controls)
        initial_slack = max(0.0, -float(np.min(initial_margins)) + 1.0e-3)
        solution = self.solver(
            x0=np.r_[initial_controls, initial_slack],
            lbx=self.lower,
            ubx=self.upper,
            lbg=self.constraint_lower,
            ubg=self.constraint_upper,
        )
        values = np.asarray(solution["x"]).reshape(-1)
        controls = values[:-1]
        slack = float(values[-1])
        return controls, slack, self.solver.stats()

    def evaluate_margins(self, controls: np.ndarray) -> np.ndarray:
        return np.asarray(self.margin_function(controls)).reshape(-1)


def deterministic_starts(physical: Any, count: int) -> list[np.ndarray]:
    lower = physical.input_lower.reshape(-1)
    upper = physical.input_upper.reshape(-1)
    recorded = np.clip(physical.seed_controls.reshape(-1), lower, upper)
    reference = np.clip(physical.input_reference.reshape(-1), lower, upper)
    starts = [recorded, reference, 0.5 * (recorded + reference)]
    rng = np.random.default_rng(20260829)
    center = 0.5 * (recorded + reference)
    span = upper - lower
    while len(starts) < count:
        starts.append(np.clip(center + rng.normal(0.0, 0.18, center.size) * span, lower, upper))
    return starts[:count]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--starts", type=int, default=12)
    arguments = parser.parse_args()
    if arguments.starts < 3 or arguments.starts > 64:
        raise ValueError("starts must be in [3, 64]")

    document = yaml.safe_load(arguments.snapshot.read_text(encoding="utf-8"))
    seed_primal = np.asarray(document["warm_start"]["primal"], dtype=float)
    numeric = load_numeric_oracle()
    physical = numeric.PhysicalProblem(document, seed_primal)
    oracle = ConstrainedOracle(physical)

    best: tuple[float, float, np.ndarray, np.ndarray, dict[str, Any]] | None = None
    for index, start in enumerate(deterministic_starts(physical, arguments.starts)):
        controls, slack, stats = oracle.solve(start)
        margins = oracle.evaluate_margins(controls)
        violation = max(0.0, -float(np.min(margins)))
        print(
            f"start={index} success={int(bool(stats.get('success', False)))} "
            f"status={stats.get('return_status', 'unknown')} "
            f"iterations={stats.get('iter_count', 0)} slack={slack:.12g} "
            f"physical_violation={violation:.12g}"
        )
        key = (violation, slack)
        if best is None or key < best[:2]:
            best = (violation, slack, controls.copy(), margins.copy(), stats)

    if best is None:
        raise RuntimeError("nonlinear solver produced no finite candidate")
    violation, slack, controls, margins, stats = best
    states, _ = physical.rollout(controls)
    if states is None:
        raise RuntimeError("best nonlinear controls do not produce a finite rollout")
    primal = np.r_[states.reshape(-1), controls]
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(arguments.output, primal)
    order = np.argsort(margins)[: min(8, margins.size)]
    worst = ",".join(
        f"{oracle.labels[int(index)]}={margins[int(index)]:.9g}"
        for index in order
    )
    feasible = violation <= FEASIBILITY_TOLERANCE and slack <= FEASIBILITY_TOLERANCE
    print(
        f"best_feasible={int(feasible)} best_slack={slack:.12g} "
        f"best_physical_violation={violation:.12g} "
        f"best_status={stats.get('return_status', 'unknown')} "
        f"worst={worst} output={arguments.output}"
    )
    return 0 if feasible else 1


if __name__ == "__main__":
    raise SystemExit(main())
