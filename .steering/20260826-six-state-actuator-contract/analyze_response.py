#!/usr/bin/env python3
"""Temporary Slice-6 evidence extractor for command/response timing."""

from __future__ import annotations

from bisect import bisect_left
from pathlib import Path
import sys

import numpy as np

from localization_scope.bag_reader import read_bag


def nearest(samples, time_sec):
    times = [sample.t for sample in samples]
    index = bisect_left(times, time_sec)
    index = min(max(index, 0), len(samples) - 1)
    if index > 0 and abs(samples[index - 1].t - time_sec) < abs(samples[index].t - time_sec):
        index -= 1
    return samples[index]


def nearest_trajectory_cross_track(state, trajectories):
    trajectory = nearest(trajectories, state.t)
    points = trajectory.values["points"]
    best = None
    for index, (first, second) in enumerate(zip(points, points[1:])):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-9:
            continue
        along = (
            (state.values["x"] - first[0]) * dx
            + (state.values["y"] - first[1]) * dy
        ) / length_squared
        along = min(max(along, 0.0), 1.0)
        projected_x = first[0] + along * dx
        projected_y = first[1] + along * dy
        offset_x = state.values["x"] - projected_x
        offset_y = state.values["y"] - projected_y
        distance = np.hypot(offset_x, offset_y)
        if best is None or distance < best[0]:
            signed = (dx * offset_y - dy * offset_x) / np.sqrt(length_squared)
            segment_yaw = np.arctan2(dy, dx)
            best = (distance, signed, index, segment_yaw, projected_x, projected_y)
    return best or (
        float("nan"),
        float("nan"),
        -1,
        float("nan"),
        float("nan"),
        float("nan"),
    )


def main() -> None:
    bag = read_bag(
        Path(sys.argv[1]),
        {
            "ekf_pose": "/localization/kinematic_state",
            "steering": "/vehicle/status/steering_status",
            "control": "/control/command/control_cmd",
            "vehicle_velocity": "/vehicle/status/velocity_status",
            "runtime_trajectory": "/planning/scenario_planning/trajectory",
        },
    )
    ekf = bag.series["ekf_pose"]
    steering = bag.series["steering"]
    control = bag.series["control"]
    trajectories = bag.series["runtime_trajectory"]

    speed_values = [sample.values["vx"] for sample in ekf]
    largest_drop = min(
        zip(ekf[1:], np.diff(speed_values)), key=lambda item: item[1]
    )
    print(
        f"ekf_time={ekf[0].t:.3f}..{ekf[-1].t:.3f} "
        f"speed={min(speed_values):.3f}..{max(speed_values):.3f} "
        f"largest_step_drop={largest_drop[1]:.3f}@{largest_drop[0].t:.3f} "
        f"trajectories={len(trajectories)}"
    )
    ranked_drops = sorted(
        zip(ekf[1:], np.diff(speed_values)), key=lambda item: item[1]
    )[:10]
    print(
        "largest_speed_drops="
        + ",".join(f"{drop:.3f}@{sample.t:.3f}" for sample, drop in ranked_drops)
    )
    threshold_reported = set()
    for state in ekf:
        cross_track = nearest_trajectory_cross_track(state, trajectories)[0]
        for threshold in (0.5, 1.0, 1.5, 2.0, 3.0):
            if threshold not in threshold_reported and cross_track >= threshold:
                print(
                    f"first_cross_track_ge_{threshold:.1f}="
                    f"{cross_track:.3f}@{state.t:.3f}"
                )
                threshold_reported.add(threshold)

    crash_time = None
    for previous, current in zip(ekf, ekf[1:]):
        if previous.values["vx"] > 6.0 and current.values["vx"] < 4.0:
            crash_time = current.t
            break
    print(f"duration={bag.duration_sec:.3f} crash_time={crash_time}")
    # The first wall invariant failure is followed immediately by the largest
    # single-sample speed drop in the bag (AWSIM collision penalty).  Use that
    # physical event rather than trying to mix controller and bag time origins.
    wall_failure_time = (
        float(sys.argv[2]) if len(sys.argv) > 2 else largest_drop[0].t
    )
    print(
        f"wall_failure_time_estimate={wall_failure_time:.3f}"
    )
    print(
        "time x y yaw speed yaw_rate command steering_report accel "
        "cross_track signed ref_index ref_yaw yaw_error"
    )
    for offset in np.arange(-2.0, 0.55, 0.10):
        time_sec = wall_failure_time + float(offset)
        state = nearest(ekf, time_sec)
        command = nearest(control, time_sec)
        report = nearest(steering, time_sec)
        (
            cross_track,
            signed_cross_track,
            ref_index,
            ref_yaw,
            _,
            _,
        ) = nearest_trajectory_cross_track(state, trajectories)
        yaw_error = np.arctan2(
            np.sin(state.values["yaw"] - ref_yaw),
            np.cos(state.values["yaw"] - ref_yaw),
        )
        print(
            f"{time_sec:.3f} {state.values['x']:.3f} {state.values['y']:.3f} "
            f"{state.values['yaw']:.3f} {state.values['vx']:.3f} "
            f"{state.values['yaw_rate']:.3f} "
            f"{command.values['steering']:.4f} "
            f"{report.values['steering']:.4f} "
            f"{command.values['acceleration']:.3f} "
            f"{cross_track:.3f} {signed_cross_track:.3f} "
            f"{ref_index:d} {ref_yaw:.3f} {yaw_error:.3f}"
        )
    if crash_time is not None:
        print("time speed yaw_rate command steering_report acceleration")
        for offset in np.arange(-2.0, 0.55, 0.10):
            time_sec = crash_time + float(offset)
            state = nearest(ekf, time_sec)
            command = nearest(control, time_sec)
            report = nearest(steering, time_sec)
            print(
                f"{time_sec:.3f} {state.values['vx']:.3f} "
                f"{state.values['yaw_rate']:.3f} "
                f"{command.values['steering']:.4f} "
                f"{report.values['steering']:.4f} "
                f"{command.values['acceleration']:.3f}"
            )

    # Estimate the steady yaw/physical-steering gain before the first impact.
    end_time = crash_time - 0.2 if crash_time is not None else bag.duration_sec
    yaw_x = []
    yaw_y = []
    actuator_x = []
    actuator_y = []
    previous_report = None
    for state in ekf:
        if state.t >= end_time or abs(state.values["vx"]) < 2.0:
            continue
        report = nearest(steering, state.t)
        command = nearest(control, state.t)
        yaw_x.append(np.tan(report.values["steering"]))
        yaw_y.append(state.values["yaw_rate"] * 1.087 / abs(state.values["vx"]))
        if previous_report is not None:
            dt = report.t - previous_report.t
            if dt > 1e-4:
                actuator_x.append(command.values["steering"] - previous_report.values["steering"])
                actuator_y.append((report.values["steering"] - previous_report.values["steering"]) / dt)
        previous_report = report
    yaw_gain = float(np.dot(yaw_x, yaw_y) / np.dot(yaw_x, yaw_x))
    inverse_tau = float(np.dot(actuator_x, actuator_y) / np.dot(actuator_x, actuator_x))
    print(f"estimated_yaw_gain={yaw_gain:.4f}")
    print(f"estimated_actuator_tau={1.0 / inverse_tau:.4f}")


if __name__ == "__main__":
    main()
