#!/usr/bin/env python3
"""Audit teacher/normal label conflicts in the exact spatial-adapter input."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
import torch

from lib.checkpoint import load_pretrained_weights, sha256_file
from lib.normal_anchor import MultiSeqNormalAnchorDataset
from lib.recurrent_policy import MultiSeqRecurrentPolicyDataset
from lib.spatial_adapter import FrozenTinyLidarSpatialResidual


SCHEMA_VERSION = 1


@dataclass(frozen=True)
class SampledCorpus:
    name: str
    scans_m: np.ndarray
    speeds_mps: np.ndarray
    corrections_rad: np.ndarray
    sample_index: np.ndarray
    sequence_index: np.ndarray
    sequence_ids: tuple[str, ...]
    sequence_lengths: tuple[int, ...]
    source_bags: tuple[str, ...]


def deterministic_subsample_indices(size: int, maximum: int) -> np.ndarray:
    if size <= 0 or maximum <= 0:
        raise ValueError("sample size and maximum must be positive")
    if size <= maximum:
        return np.arange(size, dtype=np.int64)
    return np.unique(np.linspace(0, size - 1, maximum, dtype=np.int64))


def sample_corpus(source, name: str, maximum_per_sequence: int) -> SampledCorpus:
    scans = []
    speeds = []
    corrections = []
    sample_indices = []
    sequence_indices = []
    sequence_ids = []
    sequence_lengths = []
    source_bags = []
    for sequence_index, sequence in enumerate(source.datasets):
        selected = deterministic_subsample_indices(
            len(sequence), maximum_per_sequence
        )
        scans.append(np.asarray(sequence.scans[selected], dtype=np.float32))
        speeds.append(np.asarray(sequence.speeds[selected], dtype=np.float32))
        corrections.append(
            np.asarray(
                sequence.steers[selected] - sequence.base_steers[selected],
                dtype=np.float32,
            )
        )
        sample_indices.append(selected)
        sequence_indices.append(
            np.full(len(selected), sequence_index, dtype=np.int32)
        )
        sequence_ids.append(sequence.sequence_id)
        sequence_lengths.append(len(sequence))
        source_bags.append(str(sequence.metadata["source"]["bag"]))
    return SampledCorpus(
        name=name,
        scans_m=np.concatenate(scans),
        speeds_mps=np.concatenate(speeds),
        corrections_rad=np.concatenate(corrections),
        sample_index=np.concatenate(sample_indices),
        sequence_index=np.concatenate(sequence_indices),
        sequence_ids=tuple(sequence_ids),
        sequence_lengths=tuple(sequence_lengths),
        source_bags=tuple(source_bags),
    )


def exact_adapter_features(
    model: FrozenTinyLidarSpatialResidual,
    scans_m: np.ndarray,
    speeds_mps: np.ndarray,
    batch_size: int,
) -> tuple[np.ndarray, np.ndarray]:
    if len(scans_m) != len(speeds_mps):
        raise ValueError("scan and speed arrays must be aligned")
    features = []
    predictions = []
    model.eval()
    with torch.no_grad():
        for start in range(0, len(scans_m), batch_size):
            stop = min(start + batch_size, len(scans_m))
            scans = torch.from_numpy(scans_m[start:stop])
            speeds = torch.from_numpy(speeds_mps[start:stop])
            spatial = model.normalized_spatial_features(scans)
            base = model.base_steering(scans).unsqueeze(1)
            normalized_speed = torch.clamp(
                speeds / model.max_speed_mps, 0.0, 1.5
            ).unsqueeze(1)
            features.append(
                torch.cat((spatial, normalized_speed, base), dim=1).numpy()
            )
            predictions.append(model(scans, speeds).numpy())
    result_features = np.concatenate(features).astype(np.float32, copy=False)
    result_predictions = np.concatenate(predictions).astype(
        np.float32, copy=False
    )
    if not np.all(np.isfinite(result_features)) or not np.all(
        np.isfinite(result_predictions)
    ):
        raise ValueError("candidate produced non-finite audit values")
    return result_features, result_predictions


def physical_geometry_features(
    scans_m: np.ndarray,
    speeds_mps: np.ndarray,
    base_steers_rad: np.ndarray,
    max_speed_mps: float,
    bins: int = 50,
) -> np.ndarray:
    scans = np.asarray(scans_m, dtype=np.float32)
    speeds = np.asarray(speeds_mps, dtype=np.float32)
    base = np.asarray(base_steers_rad, dtype=np.float32)
    if (
        scans.ndim != 2
        or scans.shape[1] % bins
        or speeds.shape != (len(scans),)
        or base.shape != (len(scans),)
    ):
        raise ValueError("physical geometry inputs must be aligned")
    normalized = np.clip(scans / 30.0, 0.0, 1.0)
    reshaped = normalized.reshape(len(scans), bins, scans.shape[1] // bins)
    return np.concatenate(
        (
            np.min(reshaped, axis=2),
            np.mean(reshaped, axis=2),
            np.clip(speeds / max_speed_mps, 0.0, 1.5)[:, None],
            base[:, None],
        ),
        axis=1,
    ).astype(np.float32, copy=False)


def nearest_distances(
    query: np.ndarray,
    reference: np.ndarray,
    batch_size: int = 256,
) -> tuple[np.ndarray, np.ndarray]:
    queries = np.asarray(query, dtype=np.float32)
    references = np.asarray(reference, dtype=np.float32)
    if (
        queries.ndim != 2
        or references.ndim != 2
        or queries.shape[1] != references.shape[1]
        or len(references) == 0
    ):
        raise ValueError("nearest-neighbour arrays must share a non-empty dimension")
    reference_norm = np.sum(references * references, axis=1)
    nearest_indices = []
    nearest_values = []
    for start in range(0, len(queries), batch_size):
        values = queries[start : start + batch_size]
        distance2 = (
            np.sum(values * values, axis=1, keepdims=True)
            + reference_norm[None, :]
            - 2.0 * values @ references.T
        )
        np.maximum(distance2, 0.0, out=distance2)
        indices = np.argmin(distance2, axis=1)
        nearest_indices.append(indices)
        nearest_values.append(
            np.sqrt(distance2[np.arange(len(values)), indices])
        )
    return (
        np.concatenate(nearest_indices).astype(np.int64, copy=False),
        np.concatenate(nearest_values).astype(np.float32, copy=False),
    )


def cross_sequence_nearest_distances(
    features: np.ndarray,
    sequence_index: np.ndarray,
    maximum_queries: int,
    batch_size: int = 256,
) -> np.ndarray:
    values = np.asarray(features, dtype=np.float32)
    identities = np.asarray(sequence_index, dtype=np.int32)
    if len(values) != len(identities) or len(np.unique(identities)) < 2:
        raise ValueError("cross-sequence baseline needs at least two sequences")
    selected = deterministic_subsample_indices(len(values), maximum_queries)
    reference_norm = np.sum(values * values, axis=1)
    nearest = []
    for start in range(0, len(selected), batch_size):
        query_indices = selected[start : start + batch_size]
        query = values[query_indices]
        distance2 = (
            np.sum(query * query, axis=1, keepdims=True)
            + reference_norm[None, :]
            - 2.0 * query @ values.T
        )
        np.maximum(distance2, 0.0, out=distance2)
        distance2[
            identities[query_indices, None] == identities[None, :]
        ] = np.inf
        minima = np.min(distance2, axis=1)
        if not np.all(np.isfinite(minima)):
            raise ValueError("cross-sequence baseline has no valid neighbour")
        nearest.append(np.sqrt(minima))
    return np.concatenate(nearest).astype(np.float32, copy=False)


def distribution_summary(values: np.ndarray) -> dict:
    array = np.asarray(values, dtype=np.float64)
    if array.ndim != 1 or len(array) == 0 or not np.all(np.isfinite(array)):
        raise ValueError("summary values must be finite and non-empty")
    return {
        "samples": int(len(array)),
        "mean": float(np.mean(array)),
        "p50": float(np.percentile(array, 50)),
        "p95": float(np.percentile(array, 95)),
        "maximum": float(np.max(array)),
    }


def conflict_summary(
    distances: np.ndarray,
    baseline: np.ndarray,
) -> dict:
    values = np.asarray(distances, dtype=np.float64)
    reference = np.asarray(baseline, dtype=np.float64)
    if len(values) == 0 or len(reference) == 0:
        raise ValueError("conflict summary requires query and baseline distances")
    p50 = float(np.percentile(reference, 50))
    p95 = float(np.percentile(reference, 95))
    return {
        "distance": distribution_summary(values),
        "within_normal_cross_run_p50_fraction": float(np.mean(values <= p50)),
        "within_normal_cross_run_p95_fraction": float(np.mean(values <= p95)),
    }


def conflict_fraction_record(distances: np.ndarray, p50: float, p95: float) -> dict:
    values = np.asarray(distances, dtype=np.float64)
    if values.ndim != 1:
        raise ValueError("conflict values must be one-dimensional")
    return {
        "samples": int(len(values)),
        "within_normal_cross_run_p50_fraction": (
            None if len(values) == 0 else float(np.mean(values <= p50))
        ),
        "within_normal_cross_run_p95_fraction": (
            None if len(values) == 0 else float(np.mean(values <= p95))
        ),
    }


def per_sequence_conflict_summary(
    distances: np.ndarray,
    query_indices: np.ndarray,
    corpus: SampledCorpus,
    baseline: np.ndarray,
    tail_samples: int,
) -> list[dict]:
    values = np.asarray(distances, dtype=np.float64)
    queries = np.asarray(query_indices, dtype=np.int64)
    reference = np.asarray(baseline, dtype=np.float64)
    if len(values) != len(queries) or len(reference) == 0 or tail_samples <= 0:
        raise ValueError("invalid grouped conflict inputs")
    if np.any(queries < 0) or np.any(queries >= len(corpus.scans_m)):
        raise ValueError("conflict query index is outside the sampled corpus")
    p50 = float(np.percentile(reference, 50))
    p95 = float(np.percentile(reference, 95))
    query_sequence = corpus.sequence_index[queries]
    query_sample = corpus.sample_index[queries]
    reports = []
    for sequence_index, sequence_id in enumerate(corpus.sequence_ids):
        selected = query_sequence == sequence_index
        sequence_values = values[selected]
        tail_start = max(0, corpus.sequence_lengths[sequence_index] - tail_samples)
        tail = selected & (query_sample >= tail_start)
        reports.append(
            {
                "sequence_id": sequence_id,
                "source_bag": corpus.source_bags[sequence_index],
                "sequence_samples": corpus.sequence_lengths[sequence_index],
                "queries": conflict_fraction_record(sequence_values, p50, p95),
                "tail_start_sample_index": tail_start,
                "tail_queries": conflict_fraction_record(values[tail], p50, p95),
            }
        )
    return reports


def nearest_conflict_examples(
    distances: np.ndarray,
    query_indices: np.ndarray,
    nearest_reference_indices: np.ndarray,
    query_corpus: SampledCorpus,
    reference_corpus: SampledCorpus,
    limit: int,
) -> list[dict]:
    values = np.asarray(distances, dtype=np.float64)
    queries = np.asarray(query_indices, dtype=np.int64)
    references = np.asarray(nearest_reference_indices, dtype=np.int64)
    if (
        len(values) != len(queries)
        or len(values) != len(references)
        or limit <= 0
    ):
        raise ValueError("invalid nearest conflict example inputs")
    records = []
    for offset in np.argsort(values)[:limit]:
        query = int(queries[offset])
        reference = int(references[offset])
        query_sequence = int(query_corpus.sequence_index[query])
        reference_sequence = int(reference_corpus.sequence_index[reference])
        records.append(
            {
                "distance": float(values[offset]),
                "teacher_sequence_id": query_corpus.sequence_ids[query_sequence],
                "teacher_source_bag": query_corpus.source_bags[query_sequence],
                "teacher_sample_index": int(query_corpus.sample_index[query]),
                "teacher_correction_rad": float(
                    query_corpus.corrections_rad[query]
                ),
                "normal_sequence_id": reference_corpus.sequence_ids[
                    reference_sequence
                ],
                "normal_source_bag": reference_corpus.source_bags[
                    reference_sequence
                ],
                "normal_sample_index": int(
                    reference_corpus.sample_index[reference]
                ),
            }
        )
    return records


def label_correction_summary(
    correction_rad: np.ndarray, material_delta_rad: float
) -> dict:
    correction = np.asarray(correction_rad, dtype=np.float64)
    if (
        correction.ndim != 1
        or len(correction) == 0
        or material_delta_rad <= 0.0
        or not np.all(np.isfinite(correction))
    ):
        raise ValueError("invalid label correction summary input")
    material = np.abs(correction) >= material_delta_rad
    return {
        "samples": int(len(correction)),
        "mean_abs_rad": float(np.mean(np.abs(correction))),
        "p95_abs_rad": float(np.percentile(np.abs(correction), 95)),
        "maximum_abs_rad": float(np.max(np.abs(correction))),
        "material_samples": int(np.count_nonzero(material)),
        "material_fraction": float(np.mean(material)),
        "left_samples": int(np.count_nonzero(correction <= -material_delta_rad)),
        "right_samples": int(np.count_nonzero(correction >= material_delta_rad)),
    }


def current_teacher_records(
    scans_m: np.ndarray,
    base_steers_rad: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    from tiny_lidar_net_controller.gap_teacher import (
        GapTeacherConfig,
        LidarPrecontactTeacher,
    )

    scans = np.asarray(scans_m, dtype=np.float32)
    base = np.asarray(base_steers_rad, dtype=np.float32)
    if scans.ndim != 2 or base.shape != (len(scans),):
        raise ValueError("teacher replay inputs must be aligned")
    teacher = LidarPrecontactTeacher(GapTeacherConfig())
    corrections = []
    reasons = []
    for scan, base_steering in zip(scans, base):
        decision = teacher.decide(scan, float(base_steering), 0.6)
        corrections.append(decision.steering_rad - decision.base_steering_rad)
        reasons.append(decision.reason)
    return (
        np.asarray(corrections, dtype=np.float32),
        np.asarray(reasons, dtype=object),
    )


def reason_counts(reasons: np.ndarray, selected: np.ndarray | None = None) -> dict:
    values = np.asarray(reasons, dtype=object)
    if values.ndim != 1:
        raise ValueError("teacher reasons must be one-dimensional")
    if selected is not None:
        mask = np.asarray(selected, dtype=bool)
        if mask.shape != values.shape:
            raise ValueError("teacher reason mask must align")
        values = values[mask]
    return dict(sorted(Counter(str(value) for value in values).items()))


def correction_summary(
    predictions: np.ndarray,
    targets: np.ndarray,
) -> dict:
    predicted = np.asarray(predictions, dtype=np.float64)
    target = np.asarray(targets, dtype=np.float64)
    if predicted.shape != target.shape or predicted.ndim != 1 or len(target) == 0:
        raise ValueError("correction arrays must be aligned and non-empty")
    return {
        "samples": int(len(target)),
        "target_mean_abs_rad": float(np.mean(np.abs(target))),
        "prediction_mean_abs_rad": float(np.mean(np.abs(predicted))),
        "prediction_mae_rad": float(np.mean(np.abs(predicted - target))),
        "prediction_sign_accuracy": float(
            np.mean(np.sign(predicted) == np.sign(target))
        ),
    }


def representation_conflict_report(
    teacher_features: np.ndarray,
    normal_features: np.ndarray,
    teacher_material_indices: np.ndarray,
    normal_query_indices: np.ndarray,
    normal_sequence_index: np.ndarray,
    baseline_queries: int,
    batch_size: int,
    teacher_corpus: SampledCorpus | None = None,
    normal_corpus: SampledCorpus | None = None,
    tail_samples: int = 200,
    focus_validation_token: str = "",
    example_limit: int = 12,
) -> dict:
    baseline = cross_sequence_nearest_distances(
        normal_features,
        normal_sequence_index,
        baseline_queries,
        batch_size,
    )
    nearest_normal_indices, teacher_to_normal = nearest_distances(
        teacher_features[teacher_material_indices],
        normal_features,
        batch_size,
    )
    _, normal_to_teacher = nearest_distances(
        normal_features[normal_query_indices],
        teacher_features[teacher_material_indices],
        batch_size,
    )
    report = {
        "normal_cross_run_baseline": distribution_summary(baseline),
        "material_teacher_to_normal": conflict_summary(
            teacher_to_normal, baseline
        ),
        "normal_to_material_teacher": conflict_summary(
            normal_to_teacher, baseline
        ),
    }
    if (teacher_corpus is None) != (normal_corpus is None):
        raise ValueError("both corpora are required for conflict provenance")
    if teacher_corpus is not None and normal_corpus is not None:
        per_sequence = per_sequence_conflict_summary(
            teacher_to_normal,
            teacher_material_indices,
            teacher_corpus,
            baseline,
            tail_samples,
        )
        matching_focus = [
            record
            for record in per_sequence
            if focus_validation_token
            and focus_validation_token in record["source_bag"]
        ]
        if focus_validation_token and len(matching_focus) != 1:
            raise ValueError("focus validation token must match one sequence")
        report.update(
            {
                "per_teacher_sequence": per_sequence,
                "focus_teacher_sequence": (
                    None if not matching_focus else matching_focus[0]
                ),
                "nearest_conflict_examples": nearest_conflict_examples(
                    teacher_to_normal,
                    teacher_material_indices,
                    nearest_normal_indices,
                    teacher_corpus,
                    normal_corpus,
                    example_limit,
                ),
            }
        )
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--teacher-dataset", type=Path, required=True)
    parser.add_argument("--normal-dataset", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--expected-candidate-sha256", default="")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    parser.add_argument("--max-per-sequence", type=int, default=1500)
    parser.add_argument("--max-material-queries", type=int, default=4096)
    parser.add_argument("--max-normal-queries", type=int, default=2048)
    parser.add_argument("--baseline-queries", type=int, default=1024)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--tail-samples", type=int, default=200)
    parser.add_argument(
        "--focus-validation-token", default="20260901-130837/d1"
    )
    parser.add_argument("--conflict-example-limit", type=int, default=12)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if min(
        args.max_per_sequence,
        args.max_material_queries,
        args.max_normal_queries,
        args.baseline_queries,
        args.batch_size,
        args.tail_samples,
        args.conflict_example_limit,
    ) <= 0:
        raise ValueError("audit sample limits must be positive")
    if args.material_delta_rad <= 0.0:
        raise ValueError("material delta must be positive")

    candidate_path = args.candidate.expanduser().resolve()
    candidate_sha = sha256_file(candidate_path)
    if (
        args.expected_candidate_sha256
        and candidate_sha != args.expected_candidate_sha256
    ):
        raise ValueError("candidate SHA-256 does not match expected identity")
    model = FrozenTinyLidarSpatialResidual(
        input_dim=750,
        hidden_dim=128,
        max_scan_range_m=30.0,
        max_abs_delta_rad=1.2,
        use_speed=True,
        use_base_steering=True,
        max_speed_mps=12.0,
        spatial_normalization="fixed_train_statistics",
        projection_dim=128,
        projection_seed=2026,
        head_architecture="signed_mixture",
    )
    candidate_provenance = load_pretrained_weights(model, candidate_path)

    teacher_root = args.teacher_dataset.expanduser().resolve()
    teacher_sources = []
    for split in ("train", "val"):
        teacher_sources.extend(
            MultiSeqRecurrentPolicyDataset(
                teacher_root / split, expected_split=split
            ).datasets
        )
    normal_root = args.normal_dataset.expanduser().resolve()
    normal_sources = []
    for split in ("train", "val"):
        normal_sources.extend(
            MultiSeqNormalAnchorDataset(
                normal_root / split, expected_split=split
            ).datasets
        )

    class Source:
        def __init__(self, datasets):
            self.datasets = datasets

    teacher = sample_corpus(
        Source(teacher_sources), "successor-teacher", args.max_per_sequence
    )
    normal = sample_corpus(
        Source(normal_sources), "production-normal", args.max_per_sequence
    )
    teacher_features, teacher_predictions = exact_adapter_features(
        model, teacher.scans_m, teacher.speeds_mps, args.batch_size
    )
    normal_features, normal_predictions = exact_adapter_features(
        model, normal.scans_m, normal.speeds_mps, args.batch_size
    )

    material_indices = np.flatnonzero(
        np.abs(teacher.corrections_rad) >= args.material_delta_rad
    )
    if len(material_indices) == 0:
        raise ValueError("teacher corpus contains no material corrections")
    material_indices = material_indices[
        deterministic_subsample_indices(
            len(material_indices), args.max_material_queries
        )
    ]
    normal_query_indices = deterministic_subsample_indices(
        len(normal_features), args.max_normal_queries
    )

    nearest_normal_indices, exact_teacher_to_normal = nearest_distances(
        teacher_features[material_indices], normal_features, args.batch_size
    )
    nearest_material_indices, _ = nearest_distances(
        normal_features[normal_query_indices],
        teacher_features[material_indices],
        args.batch_size,
    )
    teacher_geometry = physical_geometry_features(
        teacher.scans_m,
        teacher.speeds_mps,
        teacher_features[:, -1],
        model.max_speed_mps,
    )
    normal_geometry = physical_geometry_features(
        normal.scans_m,
        normal.speeds_mps,
        normal_features[:, -1],
        model.max_speed_mps,
    )
    representations = {
        "exact_adapter_input": representation_conflict_report(
            teacher_features,
            normal_features,
            material_indices,
            normal_query_indices,
            normal.sequence_index,
            args.baseline_queries,
            args.batch_size,
            teacher,
            normal,
            args.tail_samples,
            args.focus_validation_token,
            args.conflict_example_limit,
        ),
        "physical_binned_geometry": representation_conflict_report(
            teacher_geometry,
            normal_geometry,
            material_indices,
            normal_query_indices,
            normal.sequence_index,
            args.baseline_queries,
            args.batch_size,
            teacher,
            normal,
            args.tail_samples,
            args.focus_validation_token,
            args.conflict_example_limit,
        ),
    }

    teacher_current_correction, teacher_reasons = current_teacher_records(
        teacher.scans_m, teacher_features[:, -1]
    )
    normal_current_correction, normal_reasons = current_teacher_records(
        normal.scans_m, normal_features[:, -1]
    )
    exact_p50 = representations["exact_adapter_input"][
        "normal_cross_run_baseline"
    ]["p50"]
    exact_p50_conflict = exact_teacher_to_normal <= exact_p50
    normal_current_material = (
        np.abs(normal_current_correction) >= args.material_delta_rad
    )

    report = {
        "schema_version": SCHEMA_VERSION,
        "purpose": "diagnostic-only label observability audit",
        "candidate": candidate_provenance,
        "candidate_sha256": candidate_sha,
        "input_contract": {
            "spatial_normalization": "fixed_train_statistics",
            "projection_dim": 128,
            "wheel_speed": True,
            "embedded_base_steering": True,
        },
        "corpora": {
            "teacher": {
                "sequence_ids": list(teacher.sequence_ids),
                "samples": int(len(teacher.scans_m)),
                "material_samples_before_query_cap": int(
                    np.count_nonzero(
                        np.abs(teacher.corrections_rad)
                        >= args.material_delta_rad
                    )
                ),
            },
            "normal": {
                "sequence_ids": list(normal.sequence_ids),
                "samples": int(len(normal.scans_m)),
            },
        },
        "representations": representations,
        "model_behavior": {
            "material_teacher": {
                "correction": correction_summary(
                    teacher_predictions[material_indices],
                    teacher.corrections_rad[material_indices],
                ),
                "nearest_normal_prediction_mean_abs_rad": float(
                    np.mean(np.abs(normal_predictions[nearest_normal_indices]))
                ),
            },
            "production_normal": {
                "normal_prediction_mean_abs_rad": float(
                    np.mean(np.abs(normal_predictions[normal_query_indices]))
                ),
                "nearest_teacher_target_mean_abs_rad": float(
                    np.mean(
                        np.abs(
                            teacher.corrections_rad[
                                material_indices[nearest_material_indices]
                            ]
                        )
                    )
                ),
            },
        },
        "label_contract": {
            "teacher_replay": {
                "maximum_stored_correction_abs_error_rad": float(
                    np.max(
                        np.abs(
                            teacher_current_correction
                            - teacher.corrections_rad
                        )
                    )
                ),
                "aggregate": label_correction_summary(
                    teacher_current_correction, args.material_delta_rad
                ),
                "material_reason_counts": reason_counts(
                    teacher_reasons,
                    np.abs(teacher_current_correction)
                    >= args.material_delta_rad,
                ),
                "exact_p50_conflict_reason_counts": reason_counts(
                    teacher_reasons[material_indices], exact_p50_conflict
                ),
            },
            "production_normal_zero_label": {
                "stored_zero_maximum_abs_error_rad": float(
                    np.max(np.abs(normal.corrections_rad))
                ),
                "current_teacher_correction": label_correction_summary(
                    normal_current_correction, args.material_delta_rad
                ),
                "current_teacher_material_reason_counts": reason_counts(
                    normal_reasons, normal_current_material
                ),
            },
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "representations": report["representations"],
                "model_behavior": report["model_behavior"],
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
