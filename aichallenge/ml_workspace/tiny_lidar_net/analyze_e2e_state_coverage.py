#!/usr/bin/env python3
"""Audit failed E2E states against admitted production and teacher datasets.

This tool is diagnostic only.  It separates three questions before another
training run is authorized:

* Is the failed geometry outside the production training distribution?
* Does the admitted successor teacher request a material correction there?
* Do similar single-frame observations carry conflicting teacher actions?

It does not modify a checkpoint or make a production admission decision.
"""

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import sys
from typing import Any, Iterable, Optional

import numpy as np


SCHEMA_VERSION = 1
SCAN_SIZE = 750
MAX_RANGE_M = 30.0


@dataclass(frozen=True)
class Corpus:
    name: str
    scans_m: np.ndarray
    steers_rad: np.ndarray
    sequence_index: np.ndarray
    sequence_names: tuple[str, ...]
    base_steers_rad: Optional[np.ndarray] = None


@dataclass(frozen=True)
class FailureQuery:
    identity: str
    scans_m: np.ndarray
    timestamps_ns: np.ndarray
    window_source: str
    window_start_sec: float
    window_end_sec: float


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"failed to read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def clean_scans(scans: np.ndarray) -> np.ndarray:
    values = np.asarray(scans, dtype=np.float32)
    if values.ndim != 2 or values.shape[1] != SCAN_SIZE:
        raise ValueError(f"expected scan shape (N, {SCAN_SIZE}), got {values.shape}")
    values = np.nan_to_num(values, nan=0.0, posinf=MAX_RANGE_M, neginf=0.0)
    values = np.clip(values, 0.0, MAX_RANGE_M)
    if values.shape[0] == 0:
        raise ValueError("scan collection must not be empty")
    return values.astype(np.float32, copy=False)


def deterministic_subsample_indices(size: int, maximum: int) -> np.ndarray:
    if size <= 0 or maximum <= 0:
        raise ValueError("size and maximum must be positive")
    if size <= maximum:
        return np.arange(size, dtype=np.int64)
    return np.unique(
        np.linspace(0, size - 1, num=maximum, dtype=np.int64)
    )


def load_corpus(
    root: Path,
    name: str,
    max_per_sequence: int,
    require_base_steers: bool,
) -> Corpus:
    sequence_dirs = sorted(
        path.parent for path in root.rglob("metadata.json") if path.parent.is_dir()
    )
    if not sequence_dirs:
        raise ValueError(f"no dataset sequences found under {root}")

    scans = []
    steers = []
    base_steers = []
    sequence_indices = []
    sequence_names = []
    for sequence_id, sequence_dir in enumerate(sequence_dirs):
        metadata = load_json(sequence_dir / "metadata.json")
        recorded_id = str(metadata.get("sequence_id", sequence_dir.name))
        if recorded_id != sequence_dir.name:
            raise ValueError(f"sequence identity mismatch: {sequence_dir}")
        sequence_scans = clean_scans(
            np.load(sequence_dir / "scans.npy", allow_pickle=False)
        )
        sequence_steers = np.asarray(
            np.load(sequence_dir / "steers.npy", allow_pickle=False),
            dtype=np.float32,
        )
        if sequence_steers.shape != (len(sequence_scans),):
            raise ValueError(f"steering shape mismatch: {sequence_dir}")
        sequence_base = None
        if require_base_steers:
            base_path = sequence_dir / "base_steers.npy"
            if not base_path.is_file():
                raise ValueError(f"teacher corpus lacks base steering: {sequence_dir}")
            sequence_base = np.asarray(
                np.load(base_path, allow_pickle=False), dtype=np.float32
            )
            if sequence_base.shape != sequence_steers.shape:
                raise ValueError(f"base steering shape mismatch: {sequence_dir}")
        selected = deterministic_subsample_indices(
            len(sequence_scans), max_per_sequence
        )
        scans.append(sequence_scans[selected])
        steers.append(sequence_steers[selected])
        if sequence_base is not None:
            base_steers.append(sequence_base[selected])
        sequence_indices.append(
            np.full(len(selected), sequence_id, dtype=np.int32)
        )
        sequence_names.append(recorded_id)

    return Corpus(
        name=name,
        scans_m=np.concatenate(scans),
        steers_rad=np.concatenate(steers),
        sequence_index=np.concatenate(sequence_indices),
        sequence_names=tuple(sequence_names),
        base_steers_rad=(np.concatenate(base_steers) if base_steers else None),
    )


def geometry_features(scans_m: np.ndarray, bins: int = 50) -> np.ndarray:
    """Create deterministic near-obstacle-sensitive single-scan features."""
    scans = clean_scans(scans_m) / MAX_RANGE_M
    if SCAN_SIZE % bins != 0:
        raise ValueError(f"{SCAN_SIZE} beams cannot be divided into {bins} bins")
    reshaped = scans.reshape(len(scans), bins, SCAN_SIZE // bins)
    return np.concatenate(
        (np.min(reshaped, axis=2), np.mean(reshaped, axis=2)), axis=1
    ).astype(np.float32, copy=False)


def load_policy_model(checkpoint: Path):
    import torch

    from lib.checkpoint import load_pretrained_weights
    from lib.model import TinyLidarNet

    model = TinyLidarNet(input_dim=SCAN_SIZE, output_dim=2)
    provenance = load_pretrained_weights(model, checkpoint)
    model.eval()
    return model, provenance, torch


def policy_embeddings(model, torch_module, scans_m: np.ndarray) -> np.ndarray:
    import torch.nn.functional as functional

    scans = clean_scans(scans_m) / MAX_RANGE_M
    outputs = []
    with torch_module.no_grad():
        for start in range(0, len(scans), 512):
            values = torch_module.from_numpy(scans[start : start + 512]).unsqueeze(1)
            values = functional.relu(model.conv1(values))
            values = functional.relu(model.conv2(values))
            values = functional.relu(model.conv3(values))
            values = functional.relu(model.conv4(values))
            values = functional.relu(model.conv5(values))
            values = torch_module.flatten(values, start_dim=1)
            values = functional.relu(model.fc1(values))
            values = functional.relu(model.fc2(values))
            values = functional.relu(model.fc3(values))
            outputs.append(values.cpu().numpy())
    return np.concatenate(outputs).astype(np.float32, copy=False)


def squared_distances(query: np.ndarray, reference: np.ndarray) -> np.ndarray:
    query = np.asarray(query, dtype=np.float32)
    reference = np.asarray(reference, dtype=np.float32)
    if query.ndim != 2 or reference.ndim != 2 or query.shape[1] != reference.shape[1]:
        raise ValueError("query and reference features must share a dimension")
    distances = (
        np.sum(np.square(query), axis=1, keepdims=True)
        + np.sum(np.square(reference), axis=1)[None, :]
        - 2.0 * query @ reference.T
    )
    return np.maximum(distances, 0.0)


def nearest_indices(
    query: np.ndarray, reference: np.ndarray, count: int
) -> tuple[np.ndarray, np.ndarray]:
    if count <= 0 or count > len(reference):
        raise ValueError("invalid nearest-neighbour count")
    distance2 = squared_distances(query, reference)
    partition = np.argpartition(distance2, count - 1, axis=1)[:, :count]
    selected_distance2 = np.take_along_axis(distance2, partition, axis=1)
    order = np.argsort(selected_distance2, axis=1)
    indices = np.take_along_axis(partition, order, axis=1)
    distances = np.sqrt(
        np.take_along_axis(distance2, indices, axis=1), dtype=np.float32
    )
    return indices, distances


def cross_sequence_baseline(
    features: np.ndarray,
    sequence_index: np.ndarray,
    maximum_queries: int,
) -> np.ndarray:
    selected = deterministic_subsample_indices(len(features), maximum_queries)
    results = []
    for start in range(0, len(selected), 128):
        query_indices = selected[start : start + 128]
        distance2 = squared_distances(features[query_indices], features)
        for row, query_index in enumerate(query_indices):
            distance2[row, sequence_index == sequence_index[query_index]] = np.inf
        nearest = np.min(distance2, axis=1)
        if not np.all(np.isfinite(nearest)):
            raise ValueError("coverage corpus needs at least two sequences")
        results.append(np.sqrt(nearest, dtype=np.float32))
    return np.concatenate(results)


def percentile_rank(values: np.ndarray, reference_distribution: np.ndarray) -> np.ndarray:
    distribution = np.sort(np.asarray(reference_distribution, dtype=np.float64))
    return np.searchsorted(distribution, values, side="right") / len(distribution)


def distinct_sequence_neighbours(
    query_features: np.ndarray,
    reference_features: np.ndarray,
    sequence_index: np.ndarray,
    count: int,
) -> tuple[np.ndarray, np.ndarray]:
    unique_sequences = np.unique(sequence_index)
    count = min(count, len(unique_sequences))
    result_indices = np.empty((len(query_features), count), dtype=np.int64)
    result_distances = np.empty((len(query_features), count), dtype=np.float32)
    distance2 = squared_distances(query_features, reference_features)
    for row in range(len(query_features)):
        candidates = []
        for sequence in unique_sequences:
            members = np.flatnonzero(sequence_index == sequence)
            local = members[int(np.argmin(distance2[row, members]))]
            candidates.append((float(distance2[row, local]), int(local)))
        candidates.sort()
        chosen = candidates[:count]
        result_distances[row] = np.sqrt([item[0] for item in chosen])
        result_indices[row] = [item[1] for item in chosen]
    return result_indices, result_distances


def action_ambiguity(
    neighbour_indices: np.ndarray,
    neighbour_distances: np.ndarray,
    teacher_steers: np.ndarray,
    distance_limit: float,
    action_threshold_rad: float,
) -> dict[str, Any]:
    ambiguous = []
    usable_counts = []
    spreads = []
    for indices, distances in zip(neighbour_indices, neighbour_distances):
        usable = distances <= distance_limit
        labels = teacher_steers[indices[usable]]
        usable_counts.append(int(len(labels)))
        if len(labels):
            spreads.append(float(np.max(labels) - np.min(labels)))
        else:
            spreads.append(0.0)
        ambiguous.append(
            bool(
                np.any(labels >= action_threshold_rad)
                and np.any(labels <= -action_threshold_rad)
            )
        )
    return {
        "ambiguous_fraction": float(np.mean(ambiguous)),
        "usable_distinct_sequence_neighbours_mean": float(np.mean(usable_counts)),
        "teacher_steering_spread_p95_rad": float(np.percentile(spreads, 95)),
    }


def read_failure_query(
    run_dir: Path,
    domain: int,
    before_sec: float,
    after_sec: float,
    maximum_samples: int,
) -> FailureQuery:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:
        raise RuntimeError("rosbags is required inside the development container") from exc

    domain_dir = run_dir / f"d{domain}"
    bag = domain_dir / "rosbag2_autoware"
    detail = load_json(run_dir / f"d{domain}-result-details.json")
    motion = load_json(domain_dir / "e2e-run-analysis.json")
    scan_times = []
    scans = []
    velocity_times = []
    speeds = []
    with AnyReader([bag]) as reader:
        wanted = {
            "/sensing/lidar/scan",
            "/vehicle/status/velocity_status",
        }
        connections = [item for item in reader.connections if item.topic in wanted]
        if {item.topic for item in connections} != wanted:
            raise ValueError(f"failure bag lacks required topics: {bag}")
        for connection, timestamp, raw in reader.messages(connections=connections):
            message = reader.deserialize(raw, connection.msgtype)
            if connection.topic == "/sensing/lidar/scan":
                scan = np.asarray(message.ranges, dtype=np.float32)
                scans.append(scan)
                scan_times.append(timestamp)
            else:
                velocity_times.append(timestamp)
                speeds.append(float(message.longitudinal_velocity))
    scan_times_array = np.asarray(scan_times, dtype=np.int64)
    scans_array = clean_scans(np.asarray(scans, dtype=np.float32))
    velocity_times_array = np.asarray(velocity_times, dtype=np.int64)
    speeds_array = np.asarray(speeds, dtype=np.float64)
    moving = np.flatnonzero(np.abs(speeds_array) >= 1.0)
    if not len(moving):
        raise ValueError(f"failure bag never reached 1 m/s: {bag}")

    penalty_events = detail.get("penalty_events")
    if isinstance(penalty_events, list) and penalty_events:
        race_times = [
            float(event["race_time"])
            for event in penalty_events
            if isinstance(event, dict) and isinstance(event.get("race_time"), (int, float))
        ]
    else:
        race_times = []
    if race_times:
        event_ns = int(velocity_times_array[moving[0]] + min(race_times) * 1e9)
        source = "first-penalty-from-first-motion"
    else:
        stall_start = motion.get("motion", {}).get("positive_accel_stall_start_sec")
        if not isinstance(stall_start, (int, float)):
            raise ValueError(f"no penalty or positive-accel stall boundary: {run_dir}/d{domain}")
        event_ns = int(velocity_times_array[0] + float(stall_start) * 1e9)
        source = "positive-accel-stall"

    start_ns = int(event_ns - before_sec * 1e9)
    end_ns = int(event_ns + after_sec * 1e9)
    selected = np.flatnonzero((scan_times_array >= start_ns) & (scan_times_array <= end_ns))
    if not len(selected):
        raise ValueError(f"failure window contains no LiDAR scans: {run_dir}/d{domain}")
    selected = selected[deterministic_subsample_indices(len(selected), maximum_samples)]
    origin_ns = int(scan_times_array[0])
    return FailureQuery(
        identity=f"{run_dir.name}/d{domain}",
        scans_m=scans_array[selected],
        timestamps_ns=scan_times_array[selected],
        window_source=source,
        window_start_sec=float(start_ns - origin_ns) / 1e9,
        window_end_sec=float(end_ns - origin_ns) / 1e9,
    )


def teacher_decisions(checkpoint: Path, scans_m: np.ndarray) -> tuple[np.ndarray, np.ndarray, dict[str, int]]:
    from tiny_lidar_net_controller.gap_teacher import GapTeacherConfig
    from tiny_lidar_net_controller.tiny_lidar_net_controller_core import TinyLidarNetCore

    core = TinyLidarNetCore(
        input_dim=SCAN_SIZE,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(checkpoint),
        acceleration=0.6,
        control_mode="precontact_teacher",
        max_range=MAX_RANGE_M,
        gap_teacher_config=GapTeacherConfig(),
    )
    base = []
    teacher = []
    reasons: dict[str, int] = {}
    for scan in clean_scans(scans_m):
        core.process(scan)
        decision = core.last_gap_teacher_decision
        if decision is None:
            raise RuntimeError("precontact teacher produced no decision")
        base.append(decision.base_steering_rad)
        teacher.append(decision.steering_rad)
        reasons[decision.reason] = reasons.get(decision.reason, 0) + 1
    return (
        np.asarray(base, dtype=np.float32),
        np.asarray(teacher, dtype=np.float32),
        reasons,
    )


def feature_coverage_report(
    query: np.ndarray,
    reference: np.ndarray,
    reference_sequence_index: np.ndarray,
    baseline_queries: int,
) -> tuple[dict[str, Any], np.ndarray, float]:
    baseline = cross_sequence_baseline(
        reference, reference_sequence_index, baseline_queries
    )
    _, distances = nearest_indices(query, reference, 1)
    nearest = distances[:, 0]
    p95 = float(np.percentile(baseline, 95))
    ranks = percentile_rank(nearest, baseline)
    return (
        {
            "reference_cross_sequence_nn": {
                "p50": float(np.percentile(baseline, 50)),
                "p95": p95,
                "maximum": float(np.max(baseline)),
            },
            "query_nn": {
                "p50": float(np.percentile(nearest, 50)),
                "p95": float(np.percentile(nearest, 95)),
                "maximum": float(np.max(nearest)),
                "above_reference_p95_fraction": float(np.mean(nearest > p95)),
                "percentile_rank_p50": float(np.percentile(ranks, 50)),
            },
        },
        nearest,
        p95,
    )


def analyze(
    checkpoint: Path,
    production_corpus: Corpus,
    teacher_corpus: Corpus,
    queries: Iterable[FailureQuery],
    baseline_queries: int,
    distinct_neighbours: int,
    action_threshold_rad: float,
) -> dict[str, Any]:
    model, checkpoint_provenance, torch_module = load_policy_model(checkpoint)
    production_geometry = geometry_features(production_corpus.scans_m)
    teacher_geometry = geometry_features(teacher_corpus.scans_m)
    production_embedding = policy_embeddings(model, torch_module, production_corpus.scans_m)
    teacher_embedding = policy_embeddings(model, torch_module, teacher_corpus.scans_m)

    reports = []
    for query in queries:
        query_geometry = geometry_features(query.scans_m)
        query_embedding = policy_embeddings(model, torch_module, query.scans_m)
        base, teacher, reason_counts = teacher_decisions(checkpoint, query.scans_m)
        correction = teacher - base

        production_geometry_report, _, _ = feature_coverage_report(
            query_geometry,
            production_geometry,
            production_corpus.sequence_index,
            baseline_queries,
        )
        production_embedding_report, _, _ = feature_coverage_report(
            query_embedding,
            production_embedding,
            production_corpus.sequence_index,
            baseline_queries,
        )
        teacher_geometry_report, _, teacher_p95 = feature_coverage_report(
            query_geometry,
            teacher_geometry,
            teacher_corpus.sequence_index,
            baseline_queries,
        )
        neighbour_indices, neighbour_distances = distinct_sequence_neighbours(
            query_geometry,
            teacher_geometry,
            teacher_corpus.sequence_index,
            distinct_neighbours,
        )
        ambiguity = action_ambiguity(
            neighbour_indices,
            neighbour_distances,
            teacher_corpus.steers_rad,
            teacher_p95,
            action_threshold_rad,
        )
        reports.append(
            {
                "identity": query.identity,
                "sample_count": len(query.scans_m),
                "window": {
                    "source": query.window_source,
                    "start_sec_from_first_scan": query.window_start_sec,
                    "end_sec_from_first_scan": query.window_end_sec,
                },
                "production_geometry": production_geometry_report,
                "production_policy_embedding": production_embedding_report,
                "teacher_geometry": teacher_geometry_report,
                "teacher_action": {
                    "reason_counts": reason_counts,
                    "material_correction_fraction": float(
                        np.mean(np.abs(correction) >= 0.02)
                    ),
                    "mean_absolute_correction_rad": float(np.mean(np.abs(correction))),
                    "p95_absolute_correction_rad": float(
                        np.percentile(np.abs(correction), 95)
                    ),
                    "opposite_sign_fraction": float(
                        np.mean(
                            (np.abs(base) >= action_threshold_rad)
                            & (np.abs(teacher) >= action_threshold_rad)
                            & (np.sign(base) != np.sign(teacher))
                        )
                    ),
                },
                "single_frame_aliasing": ambiguity,
            }
        )

    return {
        "schema_version": SCHEMA_VERSION,
        "purpose": "diagnostic-only; does not authorize checkpoint promotion",
        "checkpoint": checkpoint_provenance,
        "corpora": {
            "production": {
                "name": production_corpus.name,
                "sample_count": len(production_corpus.scans_m),
                "sequence_count": len(production_corpus.sequence_names),
            },
            "successor_teacher": {
                "name": teacher_corpus.name,
                "sample_count": len(teacher_corpus.scans_m),
                "sequence_count": len(teacher_corpus.sequence_names),
            },
        },
        "thresholds": {
            "material_correction_rad": 0.02,
            "action_sign_rad": action_threshold_rad,
            "ood_reference": "cross-sequence nearest-neighbour p95",
        },
        "failures": reports,
    }


def parse_failure(value: str) -> tuple[Path, int]:
    try:
        run, domain_token = value.rsplit(":", 1)
        domain = int(domain_token.lower().removeprefix("d"))
    except (ValueError, AttributeError):
        raise argparse.ArgumentTypeError(
            "failure must use /path/to/run:dN syntax"
        ) from None
    if domain <= 0:
        raise argparse.ArgumentTypeError("domain must be positive")
    return Path(run), domain


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--production-dataset", type=Path, required=True)
    parser.add_argument("--teacher-dataset", type=Path, required=True)
    parser.add_argument("--failure", type=parse_failure, action="append", required=True)
    parser.add_argument("--window-before-sec", type=float, default=10.0)
    parser.add_argument("--window-after-sec", type=float, default=0.5)
    parser.add_argument("--max-query-samples", type=int, default=256)
    parser.add_argument("--max-reference-per-sequence", type=int, default=1500)
    parser.add_argument("--baseline-queries", type=int, default=512)
    parser.add_argument("--distinct-neighbours", type=int, default=8)
    parser.add_argument("--action-threshold-rad", type=float, default=0.05)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    positive_values = {
        "window-before-sec": args.window_before_sec,
        "max-query-samples": args.max_query_samples,
        "max-reference-per-sequence": args.max_reference_per_sequence,
        "baseline-queries": args.baseline_queries,
        "distinct-neighbours": args.distinct_neighbours,
        "action-threshold-rad": args.action_threshold_rad,
    }
    if any(not np.isfinite(value) or value <= 0 for value in positive_values.values()):
        parser.error("window, sample, neighbour and action thresholds must be positive")
    if not np.isfinite(args.window_after_sec) or args.window_after_sec < 0:
        parser.error("--window-after-sec must be finite and non-negative")

    checkpoint = args.checkpoint.expanduser().resolve()
    production_dataset = args.production_dataset.expanduser().resolve()
    teacher_dataset = args.teacher_dataset.expanduser().resolve()
    queries = [
        read_failure_query(
            run.expanduser().resolve(),
            domain,
            args.window_before_sec,
            args.window_after_sec,
            args.max_query_samples,
        )
        for run, domain in args.failure
    ]
    report = analyze(
        checkpoint,
        load_corpus(
            production_dataset,
            "production-training",
            args.max_reference_per_sequence,
            require_base_steers=False,
        ),
        load_corpus(
            teacher_dataset,
            "admitted-precontact-teacher",
            args.max_reference_per_sequence,
            require_base_steers=True,
        ),
        queries,
        args.baseline_queries,
        args.distinct_neighbours,
        args.action_threshold_rad,
    )
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    print(rendered, end="")
    if args.output is not None:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
