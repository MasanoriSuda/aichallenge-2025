#!/usr/bin/env python3
"""Compare scale-only and affine-centred coordinates on a frozen exact QP."""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path

import numpy as np
import osqp
import scipy.sparse


def load_audit_module():
    source = Path(__file__).parents[1] / (
        "20260828-feasible-qp-conditioning-audit/analyze_conditioning.py"
    )
    spec = importlib.util.spec_from_file_location("conditioning_audit", source)
    if spec is None or spec.loader is None:
        raise RuntimeError("conditioning audit module unavailable")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def affine_box_coordinates(payload: dict) -> tuple[np.ndarray, np.ndarray]:
    request = payload["assembly_request"]
    lower = np.asarray(
        request["state_lower"] + request["input_lower"], dtype=float
    )
    upper = np.asarray(
        request["state_upper"] + request["input_upper"], dtype=float
    )
    origin = np.zeros_like(lower)
    scale = np.ones_like(lower)
    two_sided = np.isfinite(lower) & np.isfinite(upper)
    origin[two_sided] = 0.5 * (lower[two_sided] + upper[two_sided])
    half_width = np.zeros_like(lower)
    half_width[two_sided] = 0.5 * (
        upper[two_sided] - lower[two_sided]
    )
    movable = two_sided & (half_width > 0.0)
    scale[movable] = half_width[movable]
    return origin, scale


def solve(P, q, A, lower, upper, maximum_iterations: int):
    solver = osqp.OSQP()
    solver.setup(
        P=P,
        q=q,
        A=A,
        l=lower,
        u=upper,
        eps_abs=1.0e-3,
        eps_rel=0.0,
        max_iter=maximum_iterations,
        scaling=0,
        warm_starting=False,
        polishing=False,
        verbose=False,
    )
    return solver.solve()


def report(path: Path) -> None:
    audit = load_audit_module()
    payload, P, q, A, lower, upper, recorded_scale = audit.load(path)
    row_scale = audit.row_scales(lower, upper)
    S = scipy.sparse.diags(row_scale)

    recorded_D = scipy.sparse.diags(recorded_scale)
    recorded = solve(
        (recorded_D @ P @ recorded_D).tocsc(),
        q * recorded_scale,
        (S @ A @ recorded_D).tocsc(),
        lower * row_scale,
        upper * row_scale,
        4000,
    )

    origin, affine_scale = affine_box_coordinates(payload)
    D = scipy.sparse.diags(affine_scale)
    symmetric_P = audit.symmetric_quadratic(P)
    shifted = A @ origin
    affine_lower = lower - shifted
    affine_upper = upper - shifted
    affine = solve(
        (D @ P @ D).tocsc(),
        affine_scale * (symmetric_P @ origin + q),
        (S @ A @ D).tocsc(),
        affine_lower * row_scale,
        affine_upper * row_scale,
        4000,
    )
    affine_long = solve(
        (D @ P @ D).tocsc(),
        affine_scale * (symmetric_P @ origin + q),
        (S @ A @ D).tocsc(),
        affine_lower * row_scale,
        affine_upper * row_scale,
        50000,
    )

    print(f"snapshot={path}")
    print(
        "recorded="
        f"{recorded.info.status}/iter:{recorded.info.iter}/"
        f"prim:{recorded.info.prim_res:.9g}/dual:{recorded.info.dual_res:.9g}"
    )
    print(
        "affine_scale="
        f"min:{affine_scale.min():.9g}/max:{affine_scale.max():.9g}/"
        f"ratio:{affine_scale.max() / affine_scale.min():.9g}"
    )
    print(
        "affine="
        f"{affine.info.status}/iter:{affine.info.iter}/"
        f"prim:{affine.info.prim_res:.9g}/dual:{affine.info.dual_res:.9g}"
    )
    print(
        "affine_50000="
        f"{affine_long.info.status}/iter:{affine_long.info.iter}/"
        f"prim:{affine_long.info.prim_res:.9g}/dual:{affine_long.info.dual_res:.9g}"
    )
    if affine.x is not None:
        physical = origin + affine_scale * affine.x
        values = A @ physical
        violation = np.maximum(lower - values, 0.0)
        violation = np.maximum(violation, np.maximum(values - upper, 0.0))
        objective = 0.5 * physical @ (symmetric_P @ physical) + q @ physical
        print(
            f"affine_physical=max_violation:{violation.max():.9g}/"
            f"objective:{objective:.12g}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshots", nargs="+", type=Path)
    for snapshot in parser.parse_args().snapshots:
        report(snapshot)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
