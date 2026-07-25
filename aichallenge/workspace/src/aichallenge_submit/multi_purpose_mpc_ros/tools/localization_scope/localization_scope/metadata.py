"""Run metadata loading, validation, and template generation."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
from typing import Any


SCHEMA_VERSION = "1.0"
TEST_TYPES = {
    "stationary",
    "straight",
    "constant_turn",
    "accel_brake",
    "lap",
    "race",
    "custom",
}
SPEED_PROFILES = {"low", "medium", "medium_high", "high", "custom", "unspecified"}

DEFAULT_TOPIC_MAPPING = {
    "clock": "/clock",
    "gnss_fix": "/sensing/gnss/nav_sat_fix",
    "gnss_pose": "/sensing/gnss/pose_with_covariance",
    "imu_raw": "/sensing/imu/imu_raw",
    "imu_corrected": "/sensing/imu/imu_data",
    "vehicle_velocity": "/vehicle/status/velocity_status",
    "steering": "/vehicle/status/steering_status",
    "ekf_input_pose": "/localization/imu_gnss_poser/pose_with_covariance",
    "twist_raw": "/localization/twist_estimator/twist_with_covariance_raw",
    "twist": "/localization/twist_estimator/twist_with_covariance",
    "ekf_pose": "/localization/kinematic_state",
    "runtime_trajectory": "/planning/scenario_planning/trajectory",
    "control": "/control/command/control_cmd",
}

DEFAULT_METADATA: dict[str, Any] = {
    "schema_version": SCHEMA_VERSION,
    "run": {
        "run_id": None,
        "display_name": "Unnamed run",
        "test_type": "lap",
        "comparison_group": None,
        "variant": None,
        "tags": [],
        "notes": "",
    },
    "awsim": {
        "version": "unknown",
        "build_id": None,
        "image_digest": None,
    },
    "experiment": {
        "speed_profile": "unspecified",
        "target_speed_mps": None,
        "vehicle_id": None,
        "lap_count": None,
    },
    "trajectory": {
        "path": None,
        "topic": DEFAULT_TOPIC_MAPPING["runtime_trajectory"],
        "record_topic": True,
        "circular": True,
    },
    "configs": {
        "mpc": None,
        "localization": [],
        "launch": None,
        "overrides": {},
    },
    "topics": deepcopy(DEFAULT_TOPIC_MAPPING),
    "analysis": {
        "speed_bands_mps": [0.0, 3.0, 5.0, 7.0],
        "speed_band_names": ["low", "medium", "medium_high", "high"],
        "sync_tolerance_sec": 0.5,
    },
}


class MetadataError(ValueError):
    """Raised when run metadata violates the supported contract."""


def _merge(default: Any, supplied: Any) -> Any:
    if isinstance(default, dict) and isinstance(supplied, dict):
        merged = deepcopy(default)
        for key, value in supplied.items():
            merged[key] = _merge(default.get(key), value) if key in default else deepcopy(value)
        return merged
    return deepcopy(supplied)


def validate_metadata(data: dict[str, Any]) -> list[str]:
    """Validate metadata and return non-fatal warnings."""

    if not isinstance(data, dict):
        raise MetadataError("metadata root must be a JSON object")
    if data.get("schema_version") != SCHEMA_VERSION:
        actual_version = data.get("schema_version")
        raise MetadataError(
            f"unsupported schema_version {actual_version!r}; "
            f"expected {SCHEMA_VERSION!r}"
        )

    run = data.get("run")
    if not isinstance(run, dict):
        raise MetadataError("run must be an object")
    if run.get("test_type") not in TEST_TYPES:
        raise MetadataError(
            f"run.test_type must be one of {', '.join(sorted(TEST_TYPES))}"
        )
    if not isinstance(run.get("display_name"), str) or not run["display_name"].strip():
        raise MetadataError("run.display_name must be a non-empty string")

    experiment = data.get("experiment")
    if not isinstance(experiment, dict):
        raise MetadataError("experiment must be an object")
    if experiment.get("speed_profile") not in SPEED_PROFILES:
        raise MetadataError(
            f"experiment.speed_profile must be one of {', '.join(sorted(SPEED_PROFILES))}"
        )
    target_speed = experiment.get("target_speed_mps")
    if target_speed is not None and (
        not isinstance(target_speed, (int, float)) or target_speed < 0.0
    ):
        raise MetadataError("experiment.target_speed_mps must be null or non-negative")

    topics = data.get("topics")
    if not isinstance(topics, dict):
        raise MetadataError("topics must be an object")
    for key, value in topics.items():
        if value is not None and (not isinstance(value, str) or not value.startswith("/")):
            raise MetadataError(f"topics.{key} must be null or an absolute ROS topic")

    analysis = data.get("analysis")
    if not isinstance(analysis, dict):
        raise MetadataError("analysis must be an object")
    boundaries = analysis.get("speed_bands_mps")
    names = analysis.get("speed_band_names")
    if (
        not isinstance(boundaries, list)
        or len(boundaries) < 2
        or any(not isinstance(value, (int, float)) for value in boundaries)
        or any(right <= left for left, right in zip(boundaries, boundaries[1:]))
    ):
        raise MetadataError("analysis.speed_bands_mps must be a strictly increasing list")
    if not isinstance(names, list) or len(names) != len(boundaries):
        raise MetadataError(
            "analysis.speed_band_names must contain one name per lower speed boundary"
        )

    warnings: list[str] = []
    awsim = data.get("awsim", {})
    if awsim.get("version") in (None, "", "unknown"):
        warnings.append("AWSIM version is unknown; version-to-version attribution is limited.")
    return warnings


def load_metadata(path: Path | None) -> tuple[dict[str, Any], list[str]]:
    """Load metadata or return defaults when no file was supplied."""

    supplied: dict[str, Any] = {}
    if path is not None:
        try:
            supplied = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise MetadataError(f"failed to read metadata {path}: {error}") from error
    merged = _merge(DEFAULT_METADATA, supplied)
    warnings = validate_metadata(merged)
    if path is None:
        warnings.insert(0, "No run-metadata.json was supplied; defaults are in use.")
    return merged, warnings


def write_template(path: Path, *, overwrite: bool = False) -> None:
    """Write a commented-by-example JSON template."""

    if path.exists() and not overwrite:
        raise FileExistsError(f"refusing to overwrite existing metadata: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(DEFAULT_METADATA, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
