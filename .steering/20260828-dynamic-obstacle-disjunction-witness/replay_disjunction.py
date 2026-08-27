#!/usr/bin/env python3
"""Replay alternative obstacle disjunctions on one serialized exact QP.

This tool changes only the dynamic-obstacle rows.  Dynamics, actuator limits,
wall rows and every other state/input bound remain byte-for-byte equivalent to
the frozen snapshot.  The objective is ignored because this is a feasibility
classification, not a controller tuning experiment.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import scipy.optimize
import scipy.sparse
import yaml


NX = 7
NU = 3
LATERAL = 0
LAG = 1
PROGRESS = 4


def _load(path: Path):
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    qp = payload["exact_qp"]
    matrix = qp["constraints"]
    rows = np.asarray([item[0] for item in matrix["triplets"]], dtype=int)
    columns = np.asarray([item[1] for item in matrix["triplets"]], dtype=int)
    values = np.asarray([item[2] for item in matrix["triplets"]], dtype=float)
    constraints = scipy.sparse.coo_matrix(
        (values, (rows, columns)),
        shape=(int(matrix["rows"]), int(matrix["columns"])),
    ).tolil()
    return (
        payload,
        constraints,
        np.asarray(qp["lower_bound"], dtype=float),
        np.asarray(qp["upper_bound"], dtype=float),
        np.asarray(payload["warm_start"]["primal"], dtype=float),
    )


def _solve(constraints, lower, upper):
    constraints = constraints.tocsr()
    equality = np.isfinite(lower) & np.isfinite(upper) & (lower == upper)
    lower_only = np.isfinite(lower) & ~equality
    upper_only = np.isfinite(upper) & ~equality
    inequality = scipy.sparse.vstack(
        (-constraints[lower_only], constraints[upper_only]), format="csr"
    )
    return scipy.optimize.linprog(
        np.zeros(constraints.shape[1]),
        A_ub=inequality,
        b_ub=np.concatenate((-lower[lower_only], upper[upper_only])),
        A_eq=constraints[equality],
        b_eq=lower[equality],
        bounds=[(None, None)] * constraints.shape[1],
        method="highs",
    )


def _dynamic_offset(payload) -> int:
    request = payload["assembly_request"]
    horizon = int(request["horizon_steps"])
    state_values = NX * (horizon + 1)
    input_values = NU * horizon
    steering = horizon
    progress_wall = 2 * horizon
    swept_wall = len(request["swept_lateral_wall_constraints"])
    return 2 * state_values + input_values + steering + progress_wall + swept_wall


def replay(path: Path) -> int:
    payload, matrix, lower, upper, witness = _load(path)
    old = _solve(matrix, lower, upper)
    source = payload["source"]
    stages = source["dynamic_obstacle_stages"]
    side = int(source["dynamic_obstacle_pass_side_sign"])
    if side not in (-1, 1):
        raise ValueError("snapshot does not own an explicit pass side")

    first_transition = None
    for index, stage in enumerate(stages):
        if not stage["valid"]:
            continue
        state = (index + 1) * NX
        effective_progress = witness[state + PROGRESS] + witness[state + LAG]
        stay_upper = stage["target_progress_m"] - stage["longitudinal_overlap_m"]
        signed_lateral = side * (
            witness[state + LATERAL] - stage["target_lateral_m"]
        )
        if effective_progress > stay_upper + 1e-6:
            if signed_lateral <= 1e-6:
                raise ValueError("witness crosses the selected homotopy")
            first_transition = index
            break
    if first_transition is None:
        raise ValueError("witness never requires a disjunction transition")

    dynamic_offset = _dynamic_offset(payload)
    dynamic_constraints = payload["assembly_request"]["dynamic_obstacle_constraints"]
    first_target_lateral = next(
        stage["target_lateral_m"] for stage in stages if stage["valid"]
    )
    initial_signed_separation = side * (
        witness[LATERAL] - first_target_lateral
    )
    nonworsening_matrix = matrix.copy()
    nonworsening_lower = lower.copy()
    nonworsening_upper = upper.copy()
    for row_index, obstacle in enumerate(dynamic_constraints):
        stage_index = int(obstacle["state_stage"]) - 1
        if stage_index < first_transition:
            continue
        stage = stages[stage_index]
        state = (stage_index + 1) * NX
        separation = min(
            stage["lateral_center_separation_m"], initial_signed_separation
        )
        boundary = stage["target_lateral_m"] + side * separation
        row = dynamic_offset + row_index
        nonworsening_matrix.rows[row] = [state + LATERAL]
        nonworsening_matrix.data[row] = [1.0]
        if side > 0:
            nonworsening_lower[row] = boundary
            nonworsening_upper[row] = np.inf
        else:
            nonworsening_lower[row] = -np.inf
            nonworsening_upper[row] = boundary
    nonworsening = _solve(
        nonworsening_matrix, nonworsening_lower, nonworsening_upper
    )
    full_separation_feasible_stages = []
    for candidate_transition in range(first_transition, len(stages)):
        if not stages[candidate_transition]["valid"]:
            continue
        candidate_matrix = matrix.copy()
        candidate_lower = lower.copy()
        candidate_upper = upper.copy()
        for row_index, obstacle in enumerate(dynamic_constraints):
            stage_index = int(obstacle["state_stage"]) - 1
            if stage_index < candidate_transition:
                continue
            stage = stages[stage_index]
            state = (stage_index + 1) * NX
            boundary = (
                stage["target_lateral_m"]
                + side * stage["lateral_center_separation_m"]
            )
            row = dynamic_offset + row_index
            candidate_matrix.rows[row] = [state + LATERAL]
            candidate_matrix.data[row] = [1.0]
            if side > 0:
                candidate_lower[row] = boundary
                candidate_upper[row] = np.inf
            else:
                candidate_lower[row] = -np.inf
                candidate_upper[row] = boundary
        if _solve(candidate_matrix, candidate_lower, candidate_upper).success:
            full_separation_feasible_stages.append(candidate_transition)

    for row_index, obstacle in enumerate(dynamic_constraints):
        stage_index = int(obstacle["state_stage"]) - 1
        if stage_index < first_transition:
            continue
        stage = stages[stage_index]
        state = (stage_index + 1) * NX
        signed_witness = side * (
            witness[state + LATERAL] - stage["target_lateral_m"]
        )
        if signed_witness <= 1e-6:
            raise ValueError("witness leaves selected side after transition")
        separation = min(stage["lateral_center_separation_m"], signed_witness)
        boundary = stage["target_lateral_m"] + side * separation
        row = dynamic_offset + row_index
        matrix.rows[row] = [state + LATERAL]
        matrix.data[row] = [1.0]
        if side > 0:
            lower[row] = boundary
            upper[row] = np.inf
        else:
            lower[row] = -np.inf
            upper[row] = boundary

    alternate = _solve(matrix, lower, upper)
    print(f"snapshot={path}")
    print(f"old_disjunction_feasible={int(old.success)}")
    print(f"first_witness_transition_stage={first_transition}")
    print(
        "full_separation_transition_feasible_stages="
        + ",".join(str(stage) for stage in full_separation_feasible_stages)
    )
    print(
        "initial_separation_nonworsening_feasible="
        f"{int(nonworsening.success)}"
    )
    print(f"witness_preserving_disjunction_feasible={int(alternate.success)}")
    print(f"alternate_status={alternate.status}")
    print(f"alternate_message={alternate.message}")
    return 0 if (not old.success and alternate.success) else 2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    args = parser.parse_args()
    return replay(args.snapshot)


if __name__ == "__main__":
    raise SystemExit(main())
