import importlib.util
import json
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "analyze_e2e_competition.py"
SPEC = importlib.util.spec_from_file_location("analyze_e2e_competition", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


RUNTIME_CHECKPOINT = (
    "/aichallenge/workspace/install/tiny_lidar_net_controller/share/"
    "tiny_lidar_net_controller/ckpt/tinylidarnet_weights.npy"
)


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def create_run(tmp_path: Path, domain: int = 1) -> Path:
    run_dir = tmp_path / "run"
    domain_dir = run_dir / f"d{domain}"
    write_json(
        domain_dir / "e2e-run-analysis.json",
        {
            "schema_version": 2,
            "admission": {"result": "pass"},
            "motion": {"distance_m": 1000.0, "longest_low_speed_sec": 0.0},
        },
    )
    write_json(
        run_dir / f"d{domain}-result-details.json",
        {
            "schema_version": "v3",
            "vehicle_number": domain,
            "finished": True,
            "lap_count": 3,
            "required_laps": 3,
            "min_lap_time": 40.0,
            "avg_lap_time": 41.0,
            "total_lap_time": 123.0,
            "penalty_count": 0,
            "penalty_total_seconds": 0.0,
            "penalty_events": [],
            "penalty_by_kind": {},
        },
    )
    write_json(
        run_dir / "result-summary.json",
        {
            "schema_version": "v2",
            "vehicles": [
                {
                    "vehicle_number": domain,
                    "finished": True,
                    "lap_count": 3,
                }
            ],
        },
    )
    domain_dir.joinpath("autoware.log").write_text(
        "\n".join(
            (
                f"[INFO] [launch.user]:  - tiny_lidar_ckpt_path: {RUNTIME_CHECKPOINT}",
                "[INFO] [launch.user]:  - tiny_lidar_control_mode: fixed_lidar_brake",
            )
        ),
        encoding="utf-8",
    )
    return run_dir


def analyze(run_dir: Path, checkpoint: Path | None = None) -> dict:
    return MODULE.analyze_competition(
        run_dir,
        [1],
        expected_control_mode="fixed_lidar_brake",
        expected_checkpoint_path=RUNTIME_CHECKPOINT,
        checkpoint_file=checkpoint,
        expected_checkpoint_sha256=(
            None if checkpoint is None else MODULE.sha256_file(checkpoint)
        ),
    )


def test_complete_finished_penalty_free_run_passes(tmp_path):
    run_dir = create_run(tmp_path)
    checkpoint = tmp_path / "candidate.npy"
    checkpoint.write_bytes(b"frozen-candidate")

    result = analyze(run_dir, checkpoint)

    assert result["status"] == "pass"
    assert result["domains"][0]["status"] == "pass"
    assert result["domains"][0]["race"]["lap_count"] == 3
    assert result["checkpoint_artifact"]["sha256"] == MODULE.sha256_file(
        checkpoint
    )


def test_motion_only_run_is_incomplete_not_success(tmp_path):
    run_dir = create_run(tmp_path)
    (run_dir / "result-summary.json").unlink()

    result = analyze(run_dir)

    assert result["status"] == "incomplete"
    assert result["reasons"] == ["missing-result-summary"]


def test_unfinished_race_fails_even_when_motion_passes(tmp_path):
    run_dir = create_run(tmp_path)
    detail_path = run_dir / "d1-result-details.json"
    detail = MODULE.load_json(detail_path)
    detail.update({"finished": False, "lap_count": 2})
    write_json(detail_path, detail)
    summary_path = run_dir / "result-summary.json"
    summary = MODULE.load_json(summary_path)
    summary["vehicles"][0].update({"finished": False, "lap_count": 2})
    write_json(summary_path, summary)

    result = analyze(run_dir)

    assert result["status"] == "fail"
    assert "race-not-finished" in result["domains"][0]["reasons"]


def test_penalty_limit_is_enforced(tmp_path):
    run_dir = create_run(tmp_path)
    detail_path = run_dir / "d1-result-details.json"
    detail = MODULE.load_json(detail_path)
    detail.update(
        {
            "penalty_count": 1,
            "penalty_total_seconds": 2.0,
            "penalty_events": [{"kind": "wall", "duration": 2.0}],
        }
    )
    write_json(detail_path, detail)

    result = analyze(run_dir)

    assert result["status"] == "fail"
    assert "penalty-limit-exceeded" in result["domains"][0]["reasons"]


def test_result_detail_must_match_domain_identity(tmp_path):
    run_dir = create_run(tmp_path)
    detail_path = run_dir / "d1-result-details.json"
    detail = MODULE.load_json(detail_path)
    detail["vehicle_number"] = 2
    write_json(detail_path, detail)

    result = analyze(run_dir)

    assert result["status"] == "fail"
    assert "result-detail-domain-mismatch" in result["domains"][0]["reasons"]


def test_npc_summary_duplicate_vehicle_numbers_use_detail_as_identity(tmp_path):
    run_dir = create_run(tmp_path)
    summary_path = run_dir / "result-summary.json"
    summary = MODULE.load_json(summary_path)
    summary["vehicles"].extend(
        [
            {"vehicle_number": 1, "finished": False, "lap_count": 1},
            {"vehicle_number": 1, "finished": False, "lap_count": 0},
        ]
    )
    write_json(summary_path, summary)

    result = analyze(run_dir)

    assert result["status"] == "pass"
    assert result["domains"][0]["summary_crosscheck"] == {
        "domain_entries": 3,
        "matching_race_entries": 1,
        "identity_ambiguous": True,
    }


def test_conflicting_runtime_provenance_fails(tmp_path):
    run_dir = create_run(tmp_path)
    log_path = run_dir / "d1" / "autoware.log"
    with log_path.open("a", encoding="utf-8") as stream:
        stream.write(
            "\n[INFO] [launch.user]:  - tiny_lidar_control_mode: precontact_teacher\n"
        )

    result = analyze(run_dir)

    assert result["status"] == "fail"
    assert "runtime-provenance-conflict" in result["domains"][0]["reasons"]


def test_checkpoint_hash_mismatch_fails(tmp_path):
    run_dir = create_run(tmp_path)
    checkpoint = tmp_path / "candidate.npy"
    checkpoint.write_bytes(b"unexpected")

    result = MODULE.analyze_competition(
        run_dir,
        [1],
        checkpoint_file=checkpoint,
        expected_checkpoint_sha256="0" * 64,
    )

    assert result["status"] == "fail"
    assert result["reasons"] == ["checkpoint-sha256-mismatch"]


def test_parse_domains_discovers_and_sorts_domain_directories(tmp_path):
    run_dir = tmp_path / "run"
    (run_dir / "d4").mkdir(parents=True)
    (run_dir / "d2").mkdir()
    (run_dir / "debug").mkdir()

    assert MODULE.parse_domains(run_dir, None) == [2, 4]
