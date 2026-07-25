from copy import deepcopy
import json
import math

import pytest

from localization_scope.analysis import analyze_run
from localization_scope.analysis import compare_runs
from localization_scope.metadata import DEFAULT_METADATA
from localization_scope.metadata import MetadataError
from localization_scope.metadata import load_metadata
from localization_scope.models import RunData
from localization_scope.models import Sample
from localization_scope.models import TrajectoryPoint
from localization_scope.report import write_comparison_report
from localization_scope.report import write_catalog
from localization_scope.report import write_single_report
from localization_scope.trajectory import load_trajectory_csv
from localization_scope.trajectory import project_pose


def _trajectory():
    return [
        TrajectoryPoint(0.0, 0.0, 0.0, 0.0),
        TrajectoryPoint(5.0, 5.0, 0.0, 0.0),
        TrajectoryPoint(10.0, 10.0, 0.0, 0.0),
    ]


def _metadata(name):
    value = deepcopy(DEFAULT_METADATA)
    value["run"]["display_name"] = name
    value["awsim"]["version"] = "2026.test"
    return value


def _manifest(hash_suffix):
    return {
        "repository": {"commit": f"commit-{hash_suffix}", "dirty": False},
        "trajectory": {"sha256": "trajectory"},
        "configs": {"mpc": {"sha256": "mpc"}, "localization": [], "launch": {}},
    }


def _run_data(tmp_path, cross_track):
    times = [0.0, 1.0, 2.0]
    ekf = [
        Sample(t, {"x": t * 5.0, "y": cross_track, "yaw": 0.0, "vx": 5.0})
        for t in times
    ]
    gnss = [
        Sample(t, {"x": t * 5.0, "y": 0.0, "yaw": 0.0}) for t in times
    ]
    speed = [Sample(t, {"vx": 5.0, "vy": 0.0, "yaw_rate": 0.0}) for t in times]
    topic_types = {
        DEFAULT_METADATA["topics"]["ekf_pose"]: "nav_msgs/msg/Odometry",
        DEFAULT_METADATA["topics"]["gnss_pose"]: (
            "geometry_msgs/msg/PoseWithCovarianceStamped"
        ),
        DEFAULT_METADATA["topics"]["vehicle_velocity"]: (
            "autoware_auto_vehicle_msgs/msg/VelocityReport"
        ),
    }
    topic_counts = {topic: 3 for topic in topic_types}
    return RunData(
        bag_path=tmp_path / "synthetic.mcap",
        series={"ekf_pose": ekf, "gnss_pose": gnss, "vehicle_velocity": speed},
        topic_types=topic_types,
        topic_counts=topic_counts,
        duration_sec=2.0,
    )


def test_projection_returns_signed_cross_track():
    projection = project_pose(_trajectory(), 4.0, 0.25, 0.0)
    assert projection.s_m == pytest.approx(4.0)
    assert projection.cross_track_m == pytest.approx(0.25)
    assert projection.yaw_error_rad == pytest.approx(0.0)


def test_projection_supports_circular_closing_segment():
    trajectory = [
        TrajectoryPoint(0.0, 0.0, 0.0, 0.0),
        TrajectoryPoint(1.0, 1.0, 0.0, math.pi / 2.0),
        TrajectoryPoint(2.0, 1.0, 1.0, math.pi),
        TrajectoryPoint(3.0, 0.0, 1.0, -math.pi / 2.0),
    ]
    projection = project_pose(
        trajectory, -0.1, 0.5, -math.pi / 2.0, circular=True
    )
    assert projection.segment_index == 3
    assert projection.cross_track_m == pytest.approx(-0.1)


def test_load_trajectory_csv(tmp_path):
    source = tmp_path / "trajectory.csv"
    source.write_text(
        "s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps\n"
        "0,0,0,0,0,3\n"
        "1,1,0,0,0,3\n",
        encoding="utf-8",
    )
    points = load_trajectory_csv(source)
    assert len(points) == 2
    assert points[1].s_m == pytest.approx(1.0)
    assert points[0].yaw_rad == pytest.approx(0.0)


def test_metadata_defaults_and_validation(tmp_path):
    source = tmp_path / "run-metadata.json"
    source.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "run": {"display_name": "test", "test_type": "lap"},
                "awsim": {"version": "2026.test"},
                "experiment": {"speed_profile": "medium"},
            }
        ),
        encoding="utf-8",
    )
    metadata, warnings = load_metadata(source)
    assert not warnings
    assert metadata["topics"]["ekf_pose"] == "/localization/kinematic_state"

    source.write_text('{"schema_version":"2.0"}', encoding="utf-8")
    with pytest.raises(MetadataError):
        load_metadata(source)


def test_single_and_comparison_reports(tmp_path):
    baseline = analyze_run(
        _run_data(tmp_path, 0.20),
        _metadata("baseline"),
        _manifest("a"),
        _trajectory(),
    )
    candidate = analyze_run(
        _run_data(tmp_path, 0.05),
        _metadata("candidate"),
        _manifest("b"),
        _trajectory(),
    )
    assert baseline.summary["metrics"]["ekf_cross_track_abs_p95_m"] == pytest.approx(0.20)
    comparison = compare_runs(baseline, candidate)
    cross_track = next(
        row
        for row in comparison["metrics"]
        if row["metric"] == "ekf_cross_track_abs_p95_m"
    )
    assert cross_track["verdict"] == "改善"

    single_paths = write_single_report(baseline, tmp_path / "single")
    comparison_paths = write_comparison_report(
        baseline, candidate, comparison, tmp_path / "comparison"
    )
    catalog_paths = write_catalog(
        [baseline, candidate],
        {"0:1": comparison, "1:0": compare_runs(candidate, baseline)},
        tmp_path / "catalog",
    )
    for path in (*single_paths, *comparison_paths, *catalog_paths):
        assert path.is_file()
        assert path.stat().st_size > 0
    catalog_text = catalog_paths[1].read_text(encoding="utf-8")
    assert "Baseline vs Candidate" in catalog_text
    assert 'id="baseline"' in catalog_text
    assert 'id="candidate"' in catalog_text
