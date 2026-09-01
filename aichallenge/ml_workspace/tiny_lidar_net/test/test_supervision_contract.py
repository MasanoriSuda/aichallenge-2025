import json
from pathlib import Path

import pytest

from lib.checkpoint import sha256_file
from lib.supervision import (
    admit_successful_run,
    validate_executed_teacher_certificate,
)


CHECKPOINT_SHA = "a" * 64


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def write_successful_teacher_run(root: Path) -> tuple[Path, Path]:
    run = root / "20260901-teacher-success"
    summary = run / "result-summary.json"
    detail = run / "d1-result-details.json"
    motion_path = run / "d1" / "e2e-run-analysis.json"
    bag = run / "d1" / "rosbag2_autoware"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    race = {
        "finished": True,
        "lap_count": 3,
        "required_laps": 3,
        "penalty_count": 0,
    }
    motion = {
        "longest_low_speed_sec": 0.0,
        "longest_positive_accel_stall_sec": 0.0,
    }
    write_json(summary, {"schema_version": "v2"})
    write_json(detail, {
        "schema_version": "v3",
        "vehicle_number": 1,
        **race,
    })
    write_json(motion_path, {
        "schema_version": 2,
        "admission": {"result": "pass"},
        "motion": motion,
    })
    report_path = run / "e2e-competition-analysis.json"
    write_json(report_path, {
        "schema_version": 1,
        "run_dir": f"/output/{run.name}",
        "status": "pass",
        "reasons": [],
        "expected_runtime": {
            "control_mode": "precontact_teacher",
            "checkpoint_sha256": CHECKPOINT_SHA,
        },
        "checkpoint_artifact": {"sha256": CHECKPOINT_SHA},
        "artifacts": {
            "result_summary": f"/output/{run.name}/result-summary.json",
            "result_summary_sha256": sha256_file(summary),
        },
        "domains": [{
            "domain": 1,
            "status": "pass",
            "reasons": [],
            "runtime": {
                "control_mode": "precontact_teacher",
                "conflicts": {},
            },
            "race": race,
            "motion": motion,
            "artifacts": {
                "result_detail": f"/output/{run.name}/d1-result-details.json",
                "result_detail_sha256": sha256_file(detail),
                "motion_analysis": (
                    f"/output/{run.name}/d1/e2e-run-analysis.json"
                ),
                "motion_sha256": sha256_file(motion_path),
            },
        }],
    })
    return run, bag


def test_successful_teacher_run_emits_source_bound_certificate(tmp_path):
    run, bag = write_successful_teacher_run(tmp_path)

    admitted = admit_successful_run(
        run, 1, "precontact_teacher", CHECKPOINT_SHA
    )
    certificate = admitted["outcome_certificate"]

    assert certificate["evidence_class"] == "executed_teacher_success"
    assert certificate["source_run_id"] == run.name
    assert certificate["result_detail_sha256"] == sha256_file(
        run / "d1-result-details.json"
    )
    assert validate_executed_teacher_certificate(
        certificate,
        source_bag=bag,
        checkpoint_sha256=CHECKPOINT_SHA,
    ) == admitted["outcome_certificate_sha256"]


@pytest.mark.parametrize(
    ("field", "value", "error"),
    [
        ("run_dir", "/output/other-run", "run identity"),
        ("expected_mode", "gap_teacher", "control mode"),
        ("expected_checkpoint", "b" * 64, "checkpoint expectation"),
        ("domain", 2, "domain identity"),
    ],
)
def test_successful_run_admission_rejects_identity_mismatch(
    tmp_path, field, value, error
):
    run, _ = write_successful_teacher_run(tmp_path)
    report_path = run / "e2e-competition-analysis.json"
    report = json.loads(report_path.read_text())
    if field == "run_dir":
        report["run_dir"] = value
        write_json(report_path, report)
    mode = value if field == "expected_mode" else "precontact_teacher"
    checkpoint = value if field == "expected_checkpoint" else CHECKPOINT_SHA
    domain = value if field == "domain" else 1

    with pytest.raises(ValueError, match=error):
        admit_successful_run(run, domain, mode, checkpoint)


def test_successful_run_admission_rejects_changed_artifact(tmp_path):
    run, _ = write_successful_teacher_run(tmp_path)
    detail = run / "d1-result-details.json"
    value = json.loads(detail.read_text())
    value["lap_count"] = 4
    write_json(detail, value)

    with pytest.raises(ValueError, match="result detail hash mismatch"):
        admit_successful_run(run, 1, "precontact_teacher", CHECKPOINT_SHA)


def test_embedded_certificate_rejects_wrong_source(tmp_path):
    run, _ = write_successful_teacher_run(tmp_path)
    certificate = admit_successful_run(
        run, 1, "precontact_teacher", CHECKPOINT_SHA
    )["outcome_certificate"]
    wrong_bag = tmp_path / "other-run" / "d1" / "rosbag2_autoware"

    with pytest.raises(ValueError, match="source mismatch"):
        validate_executed_teacher_certificate(
            certificate,
            source_bag=wrong_bag,
            checkpoint_sha256=CHECKPOINT_SHA,
        )
