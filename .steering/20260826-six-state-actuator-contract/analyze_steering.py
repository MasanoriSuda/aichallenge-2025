#!/usr/bin/env python3
"""Print command/measurement steering evidence around a bag time window."""

from __future__ import annotations

import argparse
from bisect import bisect_right
import math
from pathlib import Path
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument("--end", type=float, required=True)
    parser.add_argument("--step", type=float, default=0.1)
    parser.add_argument(
        "--fit-actuator",
        action="store_true",
        help="fit d(measured)/dt = gain * delayed(command-measured)",
    )
    parser.add_argument(
        "--fit-static-gain",
        action="store_true",
        help="fit measured = gain * delayed(command) + intercept",
    )
    parser.add_argument(
        "--check-pose-prediction",
        action="store_true",
        help="compare constant-yaw-rate prediction with the observed future pose",
    )
    parser.add_argument(
        "--prediction-delay",
        type=float,
        default=0.13,
        help="pose prediction horizon in seconds",
    )
    parser.add_argument(
        "--check-command-history-prediction",
        action="store_true",
        help="compare delayed steering-history bicycle prediction with future pose",
    )
    parser.add_argument("--actuator-static-gain", type=float, default=0.6969)
    parser.add_argument("--wheelbase", type=float, default=1.087)
    parser.add_argument("--yaw-response-gain", type=float, default=0.75)
    parser.add_argument("--yaw-response-time-constant", type=float, default=0.13)
    parser.add_argument(
        "--check-yaw-dynamics-prediction",
        action="store_true",
        help="compare first-order yaw dynamics projection with future pose",
    )
    parser.add_argument(
        "--fit-curvature-response",
        action="store_true",
        help="fit observed yaw-rate to delayed kinematic steering curvature",
    )
    parser.add_argument(
        "--fit-yaw-dynamics",
        action="store_true",
        help="fit yaw_rate_dot = a * delayed_kinematic_yaw_rate + b * yaw_rate",
    )
    parser.add_argument(
        "--dump-runtime-trajectory",
        action="store_true",
        help="print the latest runtime trajectory available at --start",
    )
    parser.add_argument(
        "--tools-dir",
        type=Path,
        help="localization_scope tools directory (needed when the script is bind-mounted)",
    )
    args = parser.parse_args()

    tools = args.tools_dir or (
        Path(__file__).resolve().parents[2]
        / "aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/tools/localization_scope"
    )
    sys.path.insert(0, str(tools))
    from localization_scope.bag_reader import read_bag

    run = read_bag(
        args.bag,
        {
            "steering": "/vehicle/status/steering_status",
            "control": "/control/command/control_cmd",
            "ekf_pose": "/localization/kinematic_state",
            "runtime_trajectory": "/planning/scenario_planning/trajectory",
        },
    )
    steering = run.series.get("steering", [])
    control = run.series.get("control", [])
    pose = run.series.get("ekf_pose", [])
    trajectories = run.series.get("runtime_trajectory", [])
    for warning in run.warnings:
        print(f"bag_warning,{warning}", file=sys.stderr)
    if not steering or not control or not pose:
        print(
            f"missing series steering={len(steering)} control={len(control)} pose={len(pose)}",
            file=sys.stderr,
        )
        return 2

    def at_or_before(series, times, timestamp):
        index = bisect_right(times, timestamp) - 1
        return series[max(0, index)]

    steering_times = [sample.t for sample in steering]
    control_times = [sample.t for sample in control]
    pose_times = [sample.t for sample in pose]
    trajectory_times = [sample.t for sample in trajectories]
    if args.dump_runtime_trajectory and trajectories:
        trajectory = at_or_before(trajectories, trajectory_times, args.start)
        points = trajectory.values.get("points", [])
        print("trajectory_index,x_m,y_m,yaw_rad,speed_mps,segment_m,yaw_delta_rad")
        for index, point in enumerate(points):
            segment_m = 0.0
            yaw_delta_rad = 0.0
            if index > 0:
                previous = points[index - 1]
                segment_m = math.hypot(
                    float(point[0]) - float(previous[0]),
                    float(point[1]) - float(previous[1]),
                )
                yaw_delta_rad = math.atan2(
                    math.sin(float(point[2]) - float(previous[2])),
                    math.cos(float(point[2]) - float(previous[2])),
                )
            print(
                f"trajectory,{index},{float(point[0]):.6f},{float(point[1]):.6f},"
                f"{float(point[2]):.6f},{float(point[3]):.6f},"
                f"{segment_m:.6f},{yaw_delta_rad:.6f}"
            )
    print(
        "t_s,measured_rad,command_rad,command_rate_radps,error_rad,"
        "speed_mps,yaw_rate_radps,x_m,y_m,yaw_rad,reference_index,"
        "reference_lateral_error_m,reference_heading_error_rad"
    )
    timestamp = args.start
    while timestamp <= args.end + 1e-9:
        measured = at_or_before(steering, steering_times, timestamp)
        command = at_or_before(control, control_times, timestamp)
        state = at_or_before(pose, pose_times, timestamp)
        reference_index = -1
        lateral_error_m = float("nan")
        heading_error_rad = float("nan")
        if trajectories:
            trajectory = at_or_before(trajectories, trajectory_times, timestamp)
            points = trajectory.values.get("points", [])
            if points:
                state_x = float(state.values["x"])
                state_y = float(state.values["y"])
                reference_index = min(
                    range(len(points)),
                    key=lambda index: (
                        (float(points[index][0]) - state_x) ** 2
                        + (float(points[index][1]) - state_y) ** 2
                    ),
                )
                reference = points[reference_index]
                dx = state_x - float(reference[0])
                dy = state_y - float(reference[1])
                reference_yaw = float(reference[2])
                lateral_error_m = -dx * math.sin(reference_yaw) + dy * math.cos(reference_yaw)
                heading_error_rad = math.atan2(
                    math.sin(float(state.values["yaw"]) - reference_yaw),
                    math.cos(float(state.values["yaw"]) - reference_yaw),
                )
        measured_rad = float(measured.values["steering"])
        command_rad = float(command.values["steering"])
        print(
            f"{timestamp:.3f},{measured_rad:.6f},{command_rad:.6f},"
            f"{float(command.values['steering_rate']):.6f},"
            f"{command_rad - measured_rad:.6f},"
            f"{float(state.values['vx']):.6f},{float(state.values['yaw_rate']):.6f},"
            f"{float(state.values['x']):.6f},{float(state.values['y']):.6f},"
            f"{float(state.values['yaw']):.6f},{reference_index},"
            f"{lateral_error_m:.6f},{heading_error_rad:.6f}"
        )
        timestamp += args.step
    window = [sample for sample in steering if args.start <= sample.t <= args.end]
    measured_rates = []
    for previous, current in zip(window, window[1:]):
        dt = current.t - previous.t
        if dt > 0.0:
            measured_rates.append(
                abs(float(current.values["steering"]) - float(previous.values["steering"])) / dt
            )
    if measured_rates:
        ordered = sorted(measured_rates)
        p95 = ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))]
        print(
            f"summary,measured_abs_rate_p95={p95:.6f},"
            f"measured_abs_rate_max={max(ordered):.6f},samples={len(ordered)}"
        )
    if args.fit_actuator:
        best = None
        for delay_index in range(31):
            delay_sec = 0.01 * delay_index
            errors = []
            rates = []
            for previous, current in zip(window, window[1:]):
                dt = current.t - previous.t
                if dt <= 0.005 or dt > 0.1:
                    continue
                command = at_or_before(control, control_times, previous.t - delay_sec)
                measured_rad = float(previous.values["steering"])
                error_rad = float(command.values["steering"]) - measured_rad
                measured_rate = (
                    float(current.values["steering"]) - measured_rad
                ) / dt
                if not math.isfinite(error_rad) or not math.isfinite(measured_rate):
                    continue
                if abs(error_rad) < 0.01 or abs(measured_rate) > 1.0:
                    continue
                errors.append(error_rad)
                rates.append(measured_rate)
            denominator = sum(error * error for error in errors)
            if len(errors) < 10 or denominator <= 1e-12:
                continue
            gain = sum(error * rate for error, rate in zip(errors, rates)) / denominator
            residual = sum(
                (rate - gain * error) ** 2 for error, rate in zip(errors, rates)
            )
            mean_rate = sum(rates) / len(rates)
            total = sum((rate - mean_rate) ** 2 for rate in rates)
            r_squared = 1.0 - residual / total if total > 1e-12 else float("nan")
            candidate = (r_squared, delay_sec, gain, len(errors))
            if best is None or candidate[0] > best[0]:
                best = candidate
        if best is not None:
            print(
                "actuator_fit,"
                f"delay_sec={best[1]:.3f},gain_per_sec={best[2]:.6f},"
                f"r_squared={best[0]:.6f},samples={best[3]}"
            )
    if args.fit_static_gain:
        best = None
        for delay_index in range(41):
            delay_sec = 0.01 * delay_index
            commands = []
            measurements = []
            for measured in window:
                command = at_or_before(
                    control, control_times, measured.t - delay_sec
                )
                command_rad = float(command.values["steering"])
                measured_rad = float(measured.values["steering"])
                if not math.isfinite(command_rad) or not math.isfinite(measured_rad):
                    continue
                commands.append(command_rad)
                measurements.append(measured_rad)
            if len(commands) < 10:
                continue
            command_mean = sum(commands) / len(commands)
            measured_mean = sum(measurements) / len(measurements)
            denominator = sum(
                (command - command_mean) ** 2 for command in commands
            )
            if denominator <= 1e-12:
                continue
            gain = sum(
                (command - command_mean) * (measured - measured_mean)
                for command, measured in zip(commands, measurements)
            ) / denominator
            intercept = measured_mean - gain * command_mean
            residual = sum(
                (measured - (gain * command + intercept)) ** 2
                for command, measured in zip(commands, measurements)
            )
            total = sum(
                (measured - measured_mean) ** 2 for measured in measurements
            )
            r_squared = 1.0 - residual / total if total > 1e-12 else float("nan")
            candidate = (
                r_squared, delay_sec, gain, intercept, len(commands)
            )
            if best is None or candidate[0] > best[0]:
                best = candidate
        if best is not None:
            print(
                "static_gain_fit,"
                f"delay_sec={best[1]:.3f},gain={best[2]:.6f},"
                f"intercept_rad={best[3]:.6f},r_squared={best[0]:.6f},"
                f"samples={best[4]}"
            )
    if args.check_pose_prediction:
        if not math.isfinite(args.prediction_delay) or args.prediction_delay < 0.0:
            print("prediction delay must be finite and non-negative", file=sys.stderr)
            return 2
        position_errors = []
        yaw_errors = []
        for state in pose:
            if state.t < args.start or state.t > args.end:
                continue
            future_time = state.t + args.prediction_delay
            if future_time > pose_times[-1]:
                continue
            future = at_or_before(pose, pose_times, future_time)
            x = float(state.values["x"])
            y = float(state.values["y"])
            yaw = float(state.values["yaw"])
            speed = float(state.values["vx"])
            yaw_rate = float(state.values["yaw_rate"])
            predicted_yaw = yaw + yaw_rate * args.prediction_delay
            if abs(yaw_rate) < 1e-6:
                travel = speed * args.prediction_delay
                predicted_x = x + travel * math.cos(yaw)
                predicted_y = y + travel * math.sin(yaw)
            else:
                radius = speed / yaw_rate
                predicted_x = x + radius * (math.sin(predicted_yaw) - math.sin(yaw))
                predicted_y = y - radius * (math.cos(predicted_yaw) - math.cos(yaw))
            position_errors.append(
                math.hypot(
                    predicted_x - float(future.values["x"]),
                    predicted_y - float(future.values["y"]),
                )
            )
            yaw_errors.append(
                abs(
                    math.atan2(
                        math.sin(predicted_yaw - float(future.values["yaw"])),
                        math.cos(predicted_yaw - float(future.values["yaw"])),
                    )
                )
            )
        if position_errors:
            ordered_position = sorted(position_errors)
            ordered_yaw = sorted(yaw_errors)

            def percentile(values, fraction):
                return values[min(len(values) - 1, int(fraction * len(values)))]

            print(
                "pose_prediction,"
                f"delay_sec={args.prediction_delay:.3f},"
                f"position_p50_m={percentile(ordered_position, 0.50):.6f},"
                f"position_p95_m={percentile(ordered_position, 0.95):.6f},"
                f"position_max_m={ordered_position[-1]:.6f},"
                f"yaw_p50_rad={percentile(ordered_yaw, 0.50):.6f},"
                f"yaw_p95_rad={percentile(ordered_yaw, 0.95):.6f},"
                f"yaw_max_rad={ordered_yaw[-1]:.6f},"
                f"samples={len(position_errors)}"
            )
    if args.check_command_history_prediction:
        if (
            not math.isfinite(args.prediction_delay)
            or args.prediction_delay < 0.0
            or not math.isfinite(args.actuator_static_gain)
            or args.actuator_static_gain <= 0.0
            or not math.isfinite(args.wheelbase)
            or args.wheelbase <= 0.0
        ):
            print("invalid command-history prediction parameters", file=sys.stderr)
            return 2
        position_errors = []
        yaw_errors = []
        integration_step_sec = 0.005
        for state in pose:
            if state.t < args.start or state.t > args.end:
                continue
            future_time = state.t + args.prediction_delay
            if future_time > pose_times[-1]:
                continue
            future = at_or_before(pose, pose_times, future_time)
            x = float(state.values["x"])
            y = float(state.values["y"])
            yaw = float(state.values["yaw"])
            speed = float(state.values["vx"])
            elapsed = 0.0
            while elapsed < args.prediction_delay - 1e-12:
                dt = min(integration_step_sec, args.prediction_delay - elapsed)
                plant_time = state.t + elapsed
                delayed_command = at_or_before(
                    control,
                    control_times,
                    plant_time - args.prediction_delay,
                )
                physical_steering = (
                    float(delayed_command.values["steering"])
                    * args.actuator_static_gain
                )
                yaw_rate = speed * math.tan(physical_steering) / args.wheelbase
                midpoint_yaw = yaw + 0.5 * yaw_rate * dt
                x += speed * math.cos(midpoint_yaw) * dt
                y += speed * math.sin(midpoint_yaw) * dt
                yaw += yaw_rate * dt
                elapsed += dt
            position_errors.append(
                math.hypot(
                    x - float(future.values["x"]),
                    y - float(future.values["y"]),
                )
            )
            yaw_errors.append(
                abs(
                    math.atan2(
                        math.sin(yaw - float(future.values["yaw"])),
                        math.cos(yaw - float(future.values["yaw"])),
                    )
                )
            )
        if position_errors:
            ordered_position = sorted(position_errors)
            ordered_yaw = sorted(yaw_errors)

            def percentile(values, fraction):
                return values[min(len(values) - 1, int(fraction * len(values)))]

            print(
                "command_history_pose_prediction,"
                f"delay_sec={args.prediction_delay:.3f},"
                f"position_p50_m={percentile(ordered_position, 0.50):.6f},"
                f"position_p95_m={percentile(ordered_position, 0.95):.6f},"
                f"position_max_m={ordered_position[-1]:.6f},"
                f"yaw_p50_rad={percentile(ordered_yaw, 0.50):.6f},"
                f"yaw_p95_rad={percentile(ordered_yaw, 0.95):.6f},"
                f"yaw_max_rad={ordered_yaw[-1]:.6f},"
                f"samples={len(position_errors)}"
            )
    if args.check_yaw_dynamics_prediction:
        if (
            not math.isfinite(args.prediction_delay)
            or args.prediction_delay < 0.0
            or not math.isfinite(args.actuator_static_gain)
            or args.actuator_static_gain <= 0.0
            or not math.isfinite(args.wheelbase)
            or args.wheelbase <= 0.0
            or not math.isfinite(args.yaw_response_gain)
            or args.yaw_response_gain <= 0.0
            or not math.isfinite(args.yaw_response_time_constant)
            or args.yaw_response_time_constant <= 0.0
        ):
            print("invalid yaw-dynamics prediction parameters", file=sys.stderr)
            return 2
        position_errors = []
        yaw_errors = []
        yaw_rate_errors = []
        integration_step_sec = 0.005
        for state in pose:
            if state.t < args.start or state.t > args.end:
                continue
            future_time = state.t + args.prediction_delay
            if future_time > pose_times[-1]:
                continue
            future = at_or_before(pose, pose_times, future_time)
            measured = at_or_before(steering, steering_times, state.t)
            command = at_or_before(control, control_times, state.t)
            initial_steering = float(measured.values["steering"])
            terminal_steering = (
                float(command.values["steering"]) * args.actuator_static_gain
            )
            x = float(state.values["x"])
            y = float(state.values["y"])
            yaw = float(state.values["yaw"])
            speed = float(state.values["vx"])
            yaw_rate = float(state.values["yaw_rate"])
            elapsed = 0.0
            while elapsed < args.prediction_delay - 1e-12:
                dt = min(integration_step_sec, args.prediction_delay - elapsed)
                fraction = (
                    min(1.0, (elapsed + 0.5 * dt) / args.prediction_delay)
                    if args.prediction_delay > 0.0
                    else 1.0
                )
                steering_rad = initial_steering + fraction * (
                    terminal_steering - initial_steering
                )
                target_yaw_rate = (
                    args.yaw_response_gain
                    * speed
                    * math.tan(steering_rad)
                    / args.wheelbase
                )
                yaw_rate += (
                    target_yaw_rate - yaw_rate
                ) * dt / args.yaw_response_time_constant
                midpoint_yaw = yaw + 0.5 * yaw_rate * dt
                x += speed * math.cos(midpoint_yaw) * dt
                y += speed * math.sin(midpoint_yaw) * dt
                yaw += yaw_rate * dt
                elapsed += dt
            position_errors.append(
                math.hypot(
                    x - float(future.values["x"]),
                    y - float(future.values["y"]),
                )
            )
            yaw_errors.append(
                abs(
                    math.atan2(
                        math.sin(yaw - float(future.values["yaw"])),
                        math.cos(yaw - float(future.values["yaw"])),
                    )
                )
            )
            yaw_rate_errors.append(
                abs(yaw_rate - float(future.values["yaw_rate"]))
            )
        if position_errors:
            ordered_position = sorted(position_errors)
            ordered_yaw = sorted(yaw_errors)
            ordered_yaw_rate = sorted(yaw_rate_errors)

            def percentile(values, fraction):
                return values[min(len(values) - 1, int(fraction * len(values)))]

            print(
                "yaw_dynamics_pose_prediction,"
                f"delay_sec={args.prediction_delay:.3f},"
                f"position_p50_m={percentile(ordered_position, 0.50):.6f},"
                f"position_p95_m={percentile(ordered_position, 0.95):.6f},"
                f"position_max_m={ordered_position[-1]:.6f},"
                f"yaw_p50_rad={percentile(ordered_yaw, 0.50):.6f},"
                f"yaw_p95_rad={percentile(ordered_yaw, 0.95):.6f},"
                f"yaw_max_rad={ordered_yaw[-1]:.6f},"
                f"yaw_rate_p95_radps={percentile(ordered_yaw_rate, 0.95):.6f},"
                f"samples={len(position_errors)}"
            )
    if args.fit_curvature_response:
        best = None
        window_pose = [sample for sample in pose if args.start <= sample.t <= args.end]
        for delay_index in range(31):
            delay_sec = 0.01 * delay_index
            kinematic_yaw_rates = []
            observed_yaw_rates = []
            for state in window_pose:
                measured = at_or_before(
                    steering, steering_times, state.t - delay_sec
                )
                speed = float(state.values["vx"])
                steering_rad = float(measured.values["steering"])
                observed_yaw_rate = float(state.values["yaw_rate"])
                if (
                    not math.isfinite(speed)
                    or not math.isfinite(steering_rad)
                    or not math.isfinite(observed_yaw_rate)
                    or abs(speed) < 4.0
                    or abs(steering_rad) < 0.03
                    or abs(observed_yaw_rate) > 3.0
                ):
                    continue
                kinematic_yaw_rate = speed * math.tan(steering_rad) / args.wheelbase
                kinematic_yaw_rates.append(kinematic_yaw_rate)
                observed_yaw_rates.append(observed_yaw_rate)
            denominator = sum(value * value for value in kinematic_yaw_rates)
            if len(kinematic_yaw_rates) < 10 or denominator <= 1e-12:
                continue
            gain = sum(
                expected * observed
                for expected, observed in zip(
                    kinematic_yaw_rates, observed_yaw_rates
                )
            ) / denominator
            residual = sum(
                (observed - gain * expected) ** 2
                for expected, observed in zip(
                    kinematic_yaw_rates, observed_yaw_rates
                )
            )
            mean_observed = sum(observed_yaw_rates) / len(observed_yaw_rates)
            total = sum(
                (observed - mean_observed) ** 2
                for observed in observed_yaw_rates
            )
            r_squared = 1.0 - residual / total if total > 1e-12 else float("nan")
            candidate = (r_squared, delay_sec, gain, len(kinematic_yaw_rates))
            if best is None or candidate[0] > best[0]:
                best = candidate
        if best is not None:
            print(
                "curvature_response_fit,"
                f"delay_sec={best[1]:.3f},gain={best[2]:.6f},"
                f"r_squared={best[0]:.6f},samples={best[3]}"
            )
    if args.fit_yaw_dynamics:
        best = None
        window_pose = [sample for sample in pose if args.start <= sample.t <= args.end]
        for delay_index in range(21):
            delay_sec = 0.01 * delay_index
            rows = []
            targets = []
            for previous, current in zip(window_pose, window_pose[1:]):
                dt = current.t - previous.t
                if dt <= 0.005 or dt > 0.1:
                    continue
                speed = float(previous.values["vx"])
                yaw_rate = float(previous.values["yaw_rate"])
                next_yaw_rate = float(current.values["yaw_rate"])
                measured = at_or_before(
                    steering, steering_times, previous.t - delay_sec
                )
                steering_rad = float(measured.values["steering"])
                yaw_rate_dot = (next_yaw_rate - yaw_rate) / dt
                if (
                    not math.isfinite(speed)
                    or not math.isfinite(steering_rad)
                    or not math.isfinite(yaw_rate)
                    or not math.isfinite(yaw_rate_dot)
                    or abs(speed) < 4.0
                    or abs(yaw_rate_dot) > 20.0
                ):
                    continue
                kinematic = speed * math.tan(steering_rad) / args.wheelbase
                rows.append((kinematic, yaw_rate))
                targets.append(yaw_rate_dot)
            if len(rows) < 10:
                continue
            sum_xx = sum(x * x for x, _ in rows)
            sum_zz = sum(z * z for _, z in rows)
            sum_xz = sum(x * z for x, z in rows)
            sum_xy = sum(x * y for (x, _), y in zip(rows, targets))
            sum_zy = sum(z * y for (_, z), y in zip(rows, targets))
            determinant = sum_xx * sum_zz - sum_xz * sum_xz
            if abs(determinant) <= 1e-12:
                continue
            coefficient_input = (
                sum_xy * sum_zz - sum_zy * sum_xz
            ) / determinant
            coefficient_state = (
                sum_zy * sum_xx - sum_xy * sum_xz
            ) / determinant
            predicted = [
                coefficient_input * x + coefficient_state * z for x, z in rows
            ]
            residual = sum(
                (target - estimate) ** 2
                for target, estimate in zip(targets, predicted)
            )
            mean_target = sum(targets) / len(targets)
            total = sum((target - mean_target) ** 2 for target in targets)
            r_squared = 1.0 - residual / total if total > 1e-12 else float("nan")
            tau_sec = (
                -1.0 / coefficient_state if coefficient_state < -1e-9 else float("nan")
            )
            steady_gain = (
                -coefficient_input / coefficient_state
                if coefficient_state < -1e-9
                else float("nan")
            )
            candidate = (
                r_squared,
                delay_sec,
                tau_sec,
                steady_gain,
                coefficient_input,
                coefficient_state,
                len(rows),
            )
            if best is None or candidate[0] > best[0]:
                best = candidate
        if best is not None:
            print(
                "yaw_dynamics_fit,"
                f"delay_sec={best[1]:.3f},tau_sec={best[2]:.6f},"
                f"steady_gain={best[3]:.6f},input_coefficient={best[4]:.6f},"
                f"state_coefficient={best[5]:.6f},r_squared={best[0]:.6f},"
                f"samples={best[6]}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
