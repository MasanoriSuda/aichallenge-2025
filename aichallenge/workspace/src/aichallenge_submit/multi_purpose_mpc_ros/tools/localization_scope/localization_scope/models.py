"""ROS-independent data models used by Localization Scope."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class Sample:
    """One normalized topic sample."""

    t: float
    values: dict[str, Any]
    source_stamp: float | None = None


@dataclass
class RunData:
    """Normalized content of one rosbag."""

    bag_path: Path
    series: dict[str, list[Sample]] = field(default_factory=dict)
    topic_types: dict[str, str] = field(default_factory=dict)
    topic_counts: dict[str, int] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    duration_sec: float = 0.0


@dataclass(frozen=True)
class TrajectoryPoint:
    """One reference trajectory point."""

    s_m: float
    x_m: float
    y_m: float
    yaw_rad: float
    curvature_radpm: float | None = None
    velocity_mps: float | None = None


@dataclass(frozen=True)
class Projection:
    """Projection of a pose onto a reference trajectory."""

    s_m: float
    cross_track_m: float
    distance_m: float
    reference_yaw_rad: float
    yaw_error_rad: float | None
    segment_index: int


@dataclass
class RunAnalysis:
    """Analysis result consumed by JSON and HTML renderers."""

    manifest: dict[str, Any]
    summary: dict[str, Any]
    plots: dict[str, Any]
    warnings: list[str] = field(default_factory=list)
