import importlib.util
import json
from pathlib import Path
import sys

import numpy as np


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "audit_e2e_supervision_outcomes.py"
)
SPEC = importlib.util.spec_from_file_location(
    "audit_e2e_supervision_outcomes", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_recorded_output_path_is_mapped_explicitly(tmp_path):
    assert MODULE.resolve_recorded_path(
        "/output/run/d2/rosbag2_autoware", tmp_path
    ) == tmp_path / "run/d2/rosbag2_autoware"
    relative = Path("local/run/d1/rosbag2_autoware")
    assert MODULE.resolve_recorded_path(str(relative), tmp_path) == relative


def test_result_outcome_is_fail_closed_and_domain_bound(tmp_path):
    missing = MODULE.read_outcome(tmp_path / "missing.json", 2)
    assert missing["classification"] == "outcome_unproven"

    detail = tmp_path / "detail.json"
    detail.write_text(json.dumps({
        "schema_version": "v3",
        "vehicle_number": 2,
        "finished": True,
        "lap_count": 3,
        "required_laps": 3,
        "penalty_count": 0,
    }))
    assert MODULE.read_outcome(detail, 2)["classification"] == "certified_success"
    assert MODULE.read_outcome(detail, 1)["classification"] == "outcome_unproven"


def test_evidence_class_distinguishes_execution_from_counterfactual():
    assert MODULE.evidence_class(
        "precontact_teacher", "certified_success"
    ) == "executed_teacher_success"
    assert MODULE.evidence_class(
        "fixed_lidar_brake", "certified_success"
    ) == "successful_alternative_policy"
    assert MODULE.evidence_class(
        "fixed_lidar_brake", "certified_failure"
    ) == "counterfactual_teacher_on_failure"
    assert MODULE.evidence_class(
        "precontact_teacher", "certified_failure"
    ) == "executed_teacher_failure"
    assert MODULE.evidence_class(None, "outcome_unproven") == "outcome_unproven"


def test_correction_summary_counts_material_actions(tmp_path):
    np.save(tmp_path / "steers.npy", np.asarray([0.0, 0.03, -0.1], dtype=np.float32))
    np.save(tmp_path / "base_steers.npy", np.asarray([0.0, 0.0, -0.07], dtype=np.float32))
    report = MODULE.correction_summary(tmp_path, 0.02)
    assert report["samples"] == 3
    assert report["material_samples"] == 2
    assert np.isclose(report["material_fraction"], 2.0 / 3.0)
