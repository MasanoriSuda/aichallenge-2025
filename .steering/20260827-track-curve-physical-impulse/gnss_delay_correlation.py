#!/usr/bin/env python3
"""Estimate GNSS content latency against raw velocity and IMU yaw rate."""

from __future__ import annotations

import importlib.util
from bisect import bisect_left
import math
from pathlib import Path
from statistics import fmean, median
import sys


def load_compare_module():
    path = Path(__file__).with_name("compare_curve_passages.py")
    spec = importlib.util.spec_from_file_location("curve_compare", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: gnss_delay_correlation.py BAG")
    compare = load_compare_module()
    series, _ = compare.read_bag(Path(sys.argv[1]))
    gnss = series["raw_gnss_pose"]
    derived: list[tuple[float, float, float]] = []
    stride = 2
    for first, second in zip(gnss, gnss[stride:]):
        dt = second.time_sec - first.time_sec
        if not 0.04 <= dt <= 0.2:
            continue
        dx = second.values["x"] - first.values["x"]
        dy = second.values["y"] - first.values["y"]
        speed = math.hypot(dx, dy) / dt
        yaw_rate = math.atan2(
            math.sin(second.values["yaw"] - first.values["yaw"]),
            math.cos(second.values["yaw"] - first.values["yaw"]),
        ) / dt
        if 1.0 <= speed <= 15.0 and abs(yaw_rate) <= 5.0:
            derived.append(((first.time_sec + second.time_sec) * 0.5, speed, yaw_rate))

    velocity_times = [sample.time_sec for sample in series["velocity_report"]]
    imu_times = [sample.time_sec for sample in series["imu"]]

    def nearest_fast(samples, times, time_sec):
        index = min(max(bisect_left(times, time_sec), 0), len(samples) - 1)
        if index > 0 and abs(times[index - 1] - time_sec) < abs(times[index] - time_sec):
            index -= 1
        return samples[index]

    print(f"derived_samples={len(derived)}", flush=True)
    for index in range(13):
        lag = index * 0.05
        speed_errors: list[float] = []
        yaw_errors: list[float] = []
        for time_sec, speed, yaw_rate in derived:
            velocity = nearest_fast(
                series["velocity_report"], velocity_times, time_sec - lag)
            imu = nearest_fast(series["imu"], imu_times, time_sec - lag)
            speed_errors.append(abs(speed - abs(velocity.values["speed"])))
            yaw_errors.append(abs(yaw_rate - imu.values["wz"]))
        print(
            f"lag={lag:.2f} speed_med={median(speed_errors):.3f} "
            f"speed_mean={fmean(speed_errors):.3f} "
            f"yaw_med={median(yaw_errors):.3f} yaw_mean={fmean(yaw_errors):.3f}",
            flush=True,
        )


if __name__ == "__main__":
    main()
