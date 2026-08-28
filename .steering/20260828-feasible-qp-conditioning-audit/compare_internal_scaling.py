#!/usr/bin/env python3
"""Compare OSQP internal scaling on frozen exact-QP snapshots."""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path

import osqp
import scipy.sparse


def load_audit_module():
    module_path = Path(__file__).with_name("analyze_conditioning.py")
    spec = importlib.util.spec_from_file_location("conditioning_audit", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError("conditioning audit module unavailable")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def solve(quadratic, linear, constraints, lower, upper, scaling: int):
    solver = osqp.OSQP()
    solver.setup(
        P=quadratic,
        q=linear,
        A=constraints,
        l=lower,
        u=upper,
        eps_abs=1.0e-3,
        eps_rel=0.0,
        max_iter=4000,
        scaling=scaling,
        warm_starting=False,
        polishing=False,
        verbose=False,
    )
    return solver.solve().info


def snapshot_paths(inputs: list[Path]) -> list[Path]:
    paths = []
    for source in inputs:
        if source.is_file():
            paths.append(source)
        else:
            paths.extend(source.rglob("snapshot.yaml"))
    return sorted(set(paths))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", type=Path, nargs="+")
    arguments = parser.parse_args()
    audit = load_audit_module()
    counts = {
        "feasible": 0,
        "scale10_solved": 0,
        "scale0_solved": 0,
        "scale0_only": 0,
        "scale10_only": 0,
        "both_unsolved_feasible": 0,
        "infeasible": 0,
        "skipped": 0,
    }
    print("snapshot,feasible,scale10_status,scale10_iter,scale0_status,scale0_iter")
    for path in snapshot_paths(arguments.inputs):
        try:
            _, quadratic, linear, constraints, lower, upper, variable_scale = audit.load(path)
        except (KeyError, TypeError, ValueError, RuntimeError):
            counts["skipped"] += 1
            continue
        lp = audit.solve_lp(constraints, lower, upper)
        if not lp.success:
            counts["infeasible"] += 1
            continue
        counts["feasible"] += 1
        row_scale = audit.row_scales(lower, upper)
        variable_diagonal = scipy.sparse.diags(variable_scale)
        row_diagonal = scipy.sparse.diags(row_scale)
        solver_quadratic = (
            variable_diagonal @ quadratic @ variable_diagonal
        ).tocsc()
        solver_linear = linear * variable_scale
        solver_constraints = (
            row_diagonal @ constraints @ variable_diagonal
        ).tocsc()
        solver_lower = lower * row_scale
        solver_upper = upper * row_scale
        scaled = solve(
            solver_quadratic,
            solver_linear,
            solver_constraints,
            solver_lower,
            solver_upper,
            10,
        )
        unscaled = solve(
            solver_quadratic,
            solver_linear,
            solver_constraints,
            solver_lower,
            solver_upper,
            0,
        )
        scaled_solved = scaled.status.lower().startswith("solved")
        unscaled_solved = unscaled.status.lower().startswith("solved")
        counts["scale10_solved"] += int(scaled_solved)
        counts["scale0_solved"] += int(unscaled_solved)
        counts["scale0_only"] += int(unscaled_solved and not scaled_solved)
        counts["scale10_only"] += int(scaled_solved and not unscaled_solved)
        counts["both_unsolved_feasible"] += int(not scaled_solved and not unscaled_solved)
        print(
            f"{path},{int(lp.success)},{scaled.status},{scaled.iter},"
            f"{unscaled.status},{unscaled.iter}"
        )
    print("summary=" + ",".join(f"{key}:{value}" for key, value in counts.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
