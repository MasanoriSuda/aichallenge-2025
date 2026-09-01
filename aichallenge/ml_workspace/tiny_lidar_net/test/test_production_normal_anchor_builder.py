import json
from pathlib import Path

import pytest

from analyze_e2e_competition import sha256_file
from build_production_normal_anchor_dataset import (
    admit_production_run,
    parse_run_spec,
    production_normal_sequence_id,
)


CHECKPOINT_SHA = "d" * 64


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def write_admitted_run(root: Path) -> Path:
    run = root / "20260901-accepted"
    summary = run / "result-summary.json"
    detail = run / "d1-result-details.json"
    motion = run / "d1" / "e2e-run-analysis.json"
    write_json(summary, {"schema_version": "v2"})
    race = {
        "finished": True,
        "lap_count": 3,
        "required_laps": 3,
        "penalty_count": 0,
    }
    motion_evidence = {
        "longest_low_speed_sec": 0.0,
        "longest_positive_accel_stall_sec": 0.0,
    }
    write_json(detail, {
        "schema_version": "v3",
        "vehicle_number": 1,
        **race,
    })
    write_json(
        motion,
        {
            "schema_version": 2,
            "admission": {"result": "pass"},
            "motion": motion_evidence,
        },
    )
    bag = run / "d1" / "rosbag2_autoware"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    report = {
        "schema_version": 1,
        "status": "pass",
        "reasons": [],
        "run_dir": str(run.resolve()),
        "expected_runtime": {
            "control_mode": "fixed_lidar_brake",
            "checkpoint_sha256": CHECKPOINT_SHA,
        },
        "checkpoint_artifact": {"sha256": CHECKPOINT_SHA},
        "artifacts": {
            "result_summary": str(summary.resolve()),
            "result_summary_sha256": sha256_file(summary),
        },
        "domains": [
            {
                "domain": 1,
                "status": "pass",
                "reasons": [],
                "runtime": {
                    "control_mode": "fixed_lidar_brake",
                    "conflicts": {},
                },
                "race": race,
                "motion": motion_evidence,
                "artifacts": {
                    "result_detail": str(detail.resolve()),
                    "result_detail_sha256": sha256_file(detail),
                    "motion_analysis": str(motion.resolve()),
                    "motion_sha256": sha256_file(motion),
                },
            }
        ],
    }
    write_json(run / "e2e-competition-analysis.json", report)
    return run


def test_parse_run_spec_binds_explicit_split(tmp_path):
    split, run = parse_run_spec(f"train:{tmp_path}")

    assert split == "train"
    assert run == tmp_path.resolve()
    with pytest.raises(ValueError, match="unsupported run split"):
        parse_run_spec(f"test:{tmp_path}")


def test_admit_production_run_accepts_frozen_zero_penalty_evidence(tmp_path):
    run = write_admitted_run(tmp_path)
    report_path = run / "e2e-competition-analysis.json"
    report = json.loads(report_path.read_text())
    report["run_dir"] = f"/output/{run.name}"
    write_json(report_path, report)

    admitted = admit_production_run(
        run,
        "val",
        1,
        "fixed_lidar_brake",
        CHECKPOINT_SHA,
    )

    assert admitted["split"] == "val"
    assert admitted["checkpoint_sha256"] == CHECKPOINT_SHA
    assert admitted["race"]["finished"] is True


@pytest.mark.parametrize(
    ("field", "value", "error"),
    [
        ("penalty_count", 1, "penalized run"),
        ("finished", False, "did not finish"),
    ],
)
def test_admit_production_run_rejects_bad_race_evidence(
    tmp_path, field, value, error
):
    run = write_admitted_run(tmp_path)
    report_path = run / "e2e-competition-analysis.json"
    report = json.loads(report_path.read_text())
    report["domains"][0]["race"][field] = value
    write_json(report_path, report)

    with pytest.raises(ValueError, match=error):
        admit_production_run(
            run, "train", 1, "fixed_lidar_brake", CHECKPOINT_SHA
        )


def test_admit_production_run_rejects_stall_and_wrong_checkpoint(tmp_path):
    run = write_admitted_run(tmp_path)
    report_path = run / "e2e-competition-analysis.json"
    report = json.loads(report_path.read_text())
    report["domains"][0]["motion"]["longest_low_speed_sec"] = 0.1
    write_json(report_path, report)

    with pytest.raises(ValueError, match="motion violation"):
        admit_production_run(
            run, "train", 1, "fixed_lidar_brake", CHECKPOINT_SHA
        )
    report["domains"][0]["motion"]["longest_low_speed_sec"] = 0.0
    write_json(report_path, report)
    with pytest.raises(ValueError, match="checkpoint expectation"):
        admit_production_run(
            run, "train", 1, "fixed_lidar_brake", "e" * 64
        )


def test_production_sequence_identity_binds_run_and_checkpoint(tmp_path):
    admitted = admit_production_run(
        write_admitted_run(tmp_path),
        "train",
        1,
        "fixed_lidar_brake",
        CHECKPOINT_SHA,
    )
    first = production_normal_sequence_id(admitted, "/scan", "/speed", 0.05)

    assert first == production_normal_sequence_id(
        admitted, "/scan", "/speed", 0.05
    )
    changed = dict(admitted, checkpoint_sha256="e" * 64)
    assert first != production_normal_sequence_id(
        changed, "/scan", "/speed", 0.05
    )
