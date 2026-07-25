"""ROS-independent localization metrics and two-run comparison."""

from __future__ import annotations

from bisect import bisect_left
from copy import deepcopy
import math
from statistics import fmean
from typing import Any

from . import __version__
from .models import RunAnalysis
from .models import RunData
from .models import Sample
from .models import TrajectoryPoint
from .trajectory import project_pose
from .trajectory import trajectory_xy


COMPARISON_THRESHOLDS = {
    "ekf_cross_track_abs_p95_m": 0.02,
    "gnss_cross_track_abs_p95_m": 0.02,
    "gnss_ekf_distance_p95_m": 0.02,
    "ekf_yaw_abs_p95_deg": 0.5,
    "stationary_speed_abs_p95_mps": 0.05,
    "stationary_imu_yaw_rate_std_radps": 0.005,
    "vehicle_ekf_speed_difference_p95_mps": 0.05,
    "imu_vehicle_yaw_rate_difference_p95_radps": 0.02,
    "steering_tracking_difference_p95_rad": 0.01,
}


def percentile(values: list[float], quantile: float) -> float | None:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return None
    if len(finite) == 1:
        return finite[0]
    position = max(0.0, min(1.0, quantile)) * (len(finite) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    fraction = position - lower
    return finite[lower] * (1.0 - fraction) + finite[upper] * fraction


def standard_deviation(values: list[float]) -> float | None:
    finite = [value for value in values if math.isfinite(value)]
    if len(finite) < 2:
        return None
    mean = fmean(finite)
    return math.sqrt(sum((value - mean) ** 2 for value in finite) / len(finite))


def _nearest_sample(
    samples: list[Sample], t: float, tolerance_sec: float
) -> Sample | None:
    if not samples:
        return None
    times = [sample.t for sample in samples]
    index = bisect_left(times, t)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index > 0:
        candidates.append(samples[index - 1])
    nearest = min(candidates, key=lambda sample: abs(sample.t - t))
    return nearest if abs(nearest.t - t) <= tolerance_sec else None


def _speed_series(data: RunData) -> tuple[str | None, list[Sample]]:
    for key in ("vehicle_velocity", "twist", "ekf_pose"):
        samples = data.series.get(key, [])
        if samples and "vx" in samples[0].values:
            return key, samples
    return None, []


def _pose_projection_series(
    samples: list[Sample],
    trajectory: list[TrajectoryPoint],
    *,
    circular: bool,
) -> list[dict[str, float]]:
    projected: list[dict[str, float]] = []
    for sample in samples:
        values = sample.values
        if not all(key in values for key in ("x", "y")):
            continue
        projection = project_pose(
            trajectory,
            float(values["x"]),
            float(values["y"]),
            float(values["yaw"]) if "yaw" in values else None,
            circular=circular,
        )
        item = {
            "t": sample.t,
            "s": projection.s_m,
            "cross_track": projection.cross_track_m,
            "distance": projection.distance_m,
        }
        if projection.yaw_error_rad is not None:
            item["yaw_error_deg"] = math.degrees(projection.yaw_error_rad)
        projected.append(item)
    return projected


def _topic_summary(
    data: RunData, topic_mapping: dict[str, str | None]
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for key, topic in topic_mapping.items():
        if not topic:
            continue
        count = data.topic_counts.get(topic, 0)
        samples = data.series.get(key, [])
        intervals = [
            right.t - left.t
            for left, right in zip(samples, samples[1:])
            if right.t >= left.t
        ]
        source_offsets = [
            sample.t - sample.source_stamp
            for sample in samples
            if sample.source_stamp is not None
        ]
        rows.append(
            {
                "key": key,
                "topic": topic,
                "type": data.topic_types.get(topic),
                "count": count,
                "rate_hz": count / data.duration_sec if data.duration_sec > 0.0 else None,
                "period_mean_sec": fmean(intervals) if intervals else None,
                "period_stddev_sec": standard_deviation(intervals),
                "max_gap_sec": max(intervals) if intervals else None,
                "record_source_offset_stddev_sec": standard_deviation(
                    source_offsets
                ),
                "available": count > 0,
            }
        )
    return rows


def _speed_band_rows(
    projections: list[dict[str, float]],
    speed_samples: list[Sample],
    boundaries: list[float],
    names: list[str],
    tolerance_sec: float,
) -> list[dict[str, Any]]:
    buckets: list[list[dict[str, float]]] = [[] for _ in names]
    for projected in projections:
        speed = _nearest_sample(speed_samples, projected["t"], tolerance_sec)
        if speed is None:
            continue
        vx = abs(float(speed.values["vx"]))
        index = max(
            0,
            min(
                len(boundaries) - 1,
                bisect_left(boundaries, vx + 1e-12) - 1,
            ),
        )
        if vx >= boundaries[-1]:
            index = len(boundaries) - 1
        buckets[index].append(projected)

    rows: list[dict[str, Any]] = []
    for index, (name, values) in enumerate(zip(names, buckets)):
        lower = boundaries[index]
        upper = boundaries[index + 1] if index + 1 < len(boundaries) else None
        cross_track = [abs(value["cross_track"]) for value in values]
        yaw_error = [
            abs(value["yaw_error_deg"])
            for value in values
            if "yaw_error_deg" in value
        ]
        rows.append(
            {
                "name": name,
                "lower_mps": lower,
                "upper_mps": upper,
                "sample_count": len(values),
                "cross_track_abs_p95_m": percentile(cross_track, 0.95),
                "yaw_abs_p95_deg": percentile(yaw_error, 0.95),
            }
        )
    return rows


def _metric_values(
    data: RunData,
    metadata: dict[str, Any],
    ekf_projection: list[dict[str, float]],
    gnss_projection: list[dict[str, float]],
    gnss_ekf_distance: list[dict[str, float]],
    speed_samples: list[Sample],
    *,
    tolerance_sec: float,
) -> dict[str, float | None]:
    ekf_cross_track = [abs(item["cross_track"]) for item in ekf_projection]
    gnss_cross_track = [abs(item["cross_track"]) for item in gnss_projection]
    ekf_yaw = [
        abs(item["yaw_error_deg"])
        for item in ekf_projection
        if "yaw_error_deg" in item
    ]
    speed_values = [
        abs(float(sample.values["vx"]))
        for sample in speed_samples
        if math.isfinite(float(sample.values["vx"]))
    ]

    def paired_abs_difference(
        left_samples: list[Sample],
        left_key: str,
        right_samples: list[Sample],
        right_key: str,
    ) -> list[float]:
        differences = []
        for left in left_samples:
            right = _nearest_sample(right_samples, left.t, tolerance_sec)
            if right is None:
                continue
            left_value = float(left.values[left_key])
            right_value = float(right.values[right_key])
            if math.isfinite(left_value) and math.isfinite(right_value):
                differences.append(abs(left_value - right_value))
        return differences

    vehicle_speed_difference = paired_abs_difference(
        data.series.get("vehicle_velocity", []),
        "vx",
        data.series.get("ekf_pose", []),
        "vx",
    )
    imu_samples = data.series.get("imu_raw") or data.series.get("imu_corrected") or []
    vehicle_yaw_samples = (
        data.series.get("vehicle_velocity") or data.series.get("twist") or []
    )
    yaw_rate_difference = paired_abs_difference(
        imu_samples,
        "yaw_rate",
        vehicle_yaw_samples,
        "yaw_rate",
    )
    steering_difference = paired_abs_difference(
        data.series.get("steering", []),
        "steering",
        data.series.get("control", []),
        "steering",
    )
    metrics: dict[str, float | None] = {
        "actual_speed_mean_mps": fmean(speed_values) if speed_values else None,
        "actual_speed_p95_mps": percentile(speed_values, 0.95),
        "actual_speed_max_mps": max(speed_values) if speed_values else None,
        "ekf_cross_track_abs_mean_m": (
            fmean(ekf_cross_track) if ekf_cross_track else None
        ),
        "ekf_cross_track_abs_p95_m": percentile(ekf_cross_track, 0.95),
        "ekf_cross_track_abs_max_m": max(ekf_cross_track) if ekf_cross_track else None,
        "ekf_yaw_abs_p95_deg": percentile(ekf_yaw, 0.95),
        "gnss_cross_track_abs_p95_m": percentile(gnss_cross_track, 0.95),
        "gnss_ekf_distance_p95_m": percentile(
            [item["distance"] for item in gnss_ekf_distance], 0.95
        ),
        "stationary_speed_abs_p95_mps": None,
        "stationary_imu_yaw_rate_std_radps": None,
        "vehicle_ekf_speed_difference_p95_mps": percentile(
            vehicle_speed_difference, 0.95
        ),
        "imu_vehicle_yaw_rate_difference_p95_radps": percentile(
            yaw_rate_difference, 0.95
        ),
        "steering_tracking_difference_p95_rad": percentile(
            steering_difference, 0.95
        ),
    }
    if metadata["run"]["test_type"] == "stationary":
        metrics["stationary_speed_abs_p95_mps"] = percentile(speed_values, 0.95)
        yaw_rates = [
            float(sample.values["yaw_rate"])
            for sample in imu_samples
            if math.isfinite(float(sample.values["yaw_rate"]))
        ]
        metrics["stationary_imu_yaw_rate_std_radps"] = standard_deviation(yaw_rates)
    return metrics


def analyze_run(
    data: RunData,
    metadata: dict[str, Any],
    input_manifest: dict[str, Any],
    trajectory: list[TrajectoryPoint],
) -> RunAnalysis:
    """Analyze one normalized run."""

    tolerance = float(metadata["analysis"]["sync_tolerance_sec"])
    circular = bool(metadata["trajectory"].get("circular", True))
    ekf_samples = data.series.get("ekf_pose", [])
    gnss_samples = data.series.get("gnss_pose", [])
    ekf_projection = _pose_projection_series(
        ekf_samples, trajectory, circular=circular
    )
    gnss_projection = _pose_projection_series(
        gnss_samples, trajectory, circular=circular
    )
    speed_source, speed_samples = _speed_series(data)

    gnss_ekf_distance: list[dict[str, float]] = []
    for sample in gnss_samples:
        ekf = _nearest_sample(ekf_samples, sample.t, tolerance)
        if ekf is None:
            continue
        dx = float(sample.values["x"]) - float(ekf.values["x"])
        dy = float(sample.values["y"]) - float(ekf.values["y"])
        gnss_ekf_distance.append(
            {"t": sample.t, "distance": math.hypot(dx, dy), "dx": dx, "dy": dy}
        )

    boundaries = [float(value) for value in metadata["analysis"]["speed_bands_mps"]]
    names = [str(value) for value in metadata["analysis"]["speed_band_names"]]
    metrics = _metric_values(
        data,
        metadata,
        ekf_projection,
        gnss_projection,
        gnss_ekf_distance,
        speed_samples,
        tolerance_sec=tolerance,
    )
    speed_bands = _speed_band_rows(
        ekf_projection,
        speed_samples,
        boundaries,
        names,
        tolerance,
    )
    topics = _topic_summary(data, metadata["topics"])

    runtime_samples = data.series.get("runtime_trajectory", [])
    runtime_points = (
        runtime_samples[-1].values.get("points", []) if runtime_samples else []
    )
    plots = {
        "trajectory_xy": trajectory_xy(trajectory, circular=circular),
        "runtime_trajectory_xy": [
            [float(point[0]), float(point[1])] for point in runtime_points
        ],
        "ekf_xy": [
            [float(sample.values["x"]), float(sample.values["y"])]
            for sample in ekf_samples
        ],
        "gnss_xy": [
            [float(sample.values["x"]), float(sample.values["y"])]
            for sample in gnss_samples
        ],
        "speed_time": [
            [sample.t, float(sample.values["vx"])] for sample in speed_samples
        ],
        "vehicle_speed_time": [
            [sample.t, float(sample.values["vx"])]
            for sample in data.series.get("vehicle_velocity", [])
        ],
        "twist_speed_time": [
            [sample.t, float(sample.values["vx"])]
            for sample in data.series.get("twist", [])
        ],
        "ekf_speed_time": [
            [sample.t, float(sample.values["vx"])]
            for sample in data.series.get("ekf_pose", [])
        ],
        "command_speed_time": [
            [sample.t, float(sample.values["speed"])]
            for sample in data.series.get("control", [])
        ],
        "imu_raw_yaw_rate_time": [
            [sample.t, float(sample.values["yaw_rate"])]
            for sample in data.series.get("imu_raw", [])
        ],
        "imu_corrected_yaw_rate_time": [
            [sample.t, float(sample.values["yaw_rate"])]
            for sample in data.series.get("imu_corrected", [])
        ],
        "vehicle_yaw_rate_time": [
            [sample.t, float(sample.values["yaw_rate"])]
            for sample in data.series.get("vehicle_velocity", [])
        ],
        "ekf_yaw_rate_time": [
            [sample.t, float(sample.values["yaw_rate"])]
            for sample in data.series.get("ekf_pose", [])
            if sample.values.get("yaw_rate") is not None
        ],
        "actual_steering_time": [
            [sample.t, float(sample.values["steering"])]
            for sample in data.series.get("steering", [])
        ],
        "command_steering_time": [
            [sample.t, float(sample.values["steering"])]
            for sample in data.series.get("control", [])
        ],
        "gnss_cov_x_time": [
            [sample.t, float(sample.values["cov_x"])]
            for sample in data.series.get("gnss_fix", [])
            if sample.values.get("cov_x") is not None
        ],
        "gnss_cov_y_time": [
            [sample.t, float(sample.values["cov_y"])]
            for sample in data.series.get("gnss_fix", [])
            if sample.values.get("cov_y") is not None
        ],
        "ekf_cross_track": [
            [item["s"], item["cross_track"]] for item in ekf_projection
        ],
        "gnss_cross_track": [
            [item["s"], item["cross_track"]] for item in gnss_projection
        ],
        "gnss_ekf_distance": [
            [item["t"], item["distance"]] for item in gnss_ekf_distance
        ],
    }
    missing_core = [
        row["topic"]
        for row in topics
        if row["key"] in {"gnss_fix", "gnss_pose", "imu_raw", "vehicle_velocity", "ekf_pose"}
        and not row["available"]
    ]
    warnings = list(data.warnings)
    if missing_core:
        warnings.append(
            "Core localization topics are absent: " + ", ".join(missing_core)
        )
    if not gnss_samples:
        warnings.append(
            "GNSS pose is unavailable; trajectory tracking and localization discrepancy "
            "cannot be separated."
        )
    if not runtime_samples:
        warnings.append(
            "Runtime trajectory is unavailable; the static CSV is used as the only reference."
        )
    warnings.append(
        "Trajectory is a target path, not ground truth; cross-track error mixes vehicle "
        "tracking and localization error."
    )

    manifest = deepcopy(metadata)
    manifest["tool"] = {"name": "localization-scope", "version": __version__}
    manifest["bag"] = {
        "path": str(data.bag_path),
        "duration_sec": data.duration_sec,
        "topic_count": len(data.topic_types),
    }
    manifest["resolved_inputs"] = input_manifest
    summary = {
        "schema_version": "1.0",
        "run_id": metadata["run"].get("run_id"),
        "display_name": metadata["run"]["display_name"],
        "duration_sec": data.duration_sec,
        "speed_source": speed_source,
        "metrics": metrics,
        "speed_bands": speed_bands,
        "topics": topics,
        "warnings": warnings,
    }
    return RunAnalysis(
        manifest=manifest,
        summary=summary,
        plots=plots,
        warnings=warnings,
    )


def _classify(
    baseline: float | None,
    candidate: float | None,
    tolerance: float,
) -> tuple[float | None, str]:
    if baseline is None or candidate is None:
        return None, "判定不能"
    delta = candidate - baseline
    if delta < -tolerance:
        return delta, "改善"
    if delta > tolerance:
        return delta, "悪化"
    return delta, "実質差なし"


def _condition_value(manifest: dict[str, Any], path: tuple[str, ...]) -> Any:
    current: Any = manifest
    for key in path:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def compare_runs(
    baseline: RunAnalysis,
    candidate: RunAnalysis,
    thresholds: dict[str, float] | None = None,
) -> dict[str, Any]:
    """Compare exactly two run analyses."""

    effective_thresholds = {**COMPARISON_THRESHOLDS, **(thresholds or {})}
    baseline_metrics = baseline.summary["metrics"]
    candidate_metrics = candidate.summary["metrics"]
    metric_rows = []
    for key, tolerance in effective_thresholds.items():
        baseline_value = baseline_metrics.get(key)
        candidate_value = candidate_metrics.get(key)
        delta, verdict = _classify(baseline_value, candidate_value, tolerance)
        metric_rows.append(
            {
                "metric": key,
                "baseline": baseline_value,
                "candidate": candidate_value,
                "delta": delta,
                "tolerance": tolerance,
                "verdict": verdict,
            }
        )

    conditions = {
        "AWSIM version": ("awsim", "version"),
        "trajectory SHA-256": (
            "resolved_inputs",
            "trajectory",
            "sha256",
        ),
        "MPC config SHA-256": (
            "resolved_inputs",
            "configs",
            "mpc",
            "sha256",
        ),
        "Git commit": ("resolved_inputs", "repository", "commit"),
        "speed profile": ("experiment", "speed_profile"),
        "target speed": ("experiment", "target_speed_mps"),
        "test type": ("run", "test_type"),
    }
    condition_rows = []
    changed_conditions = []
    for label, path in conditions.items():
        left = _condition_value(baseline.manifest, path)
        right = _condition_value(candidate.manifest, path)
        same = left == right
        condition_rows.append(
            {"condition": label, "baseline": left, "candidate": right, "same": same}
        )
        if not same:
            changed_conditions.append(label)

    warnings = []
    causal_changes = [
        name
        for name in changed_conditions
        if name in {"AWSIM version", "trajectory SHA-256", "MPC config SHA-256", "Git commit"}
    ]
    if len(causal_changes) > 1:
        warnings.append(
            "Multiple influential conditions changed ("
            + ", ".join(causal_changes)
            + "); metric deltas cannot be attributed to one cause."
        )
    return {
        "schema_version": "1.0",
        "baseline": baseline.summary["display_name"],
        "candidate": candidate.summary["display_name"],
        "metrics": metric_rows,
        "conditions": condition_rows,
        "warnings": warnings,
    }
