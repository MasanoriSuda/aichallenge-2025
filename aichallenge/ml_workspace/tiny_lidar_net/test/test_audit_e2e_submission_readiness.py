import hashlib

from audit_e2e_submission_readiness import audit_submission_readiness


def write_artifact(path, value: bytes) -> str:
    path.write_bytes(value)
    return hashlib.sha256(value).hexdigest()


def reports(
    *,
    single: str = "pass",
    peer: str = "fail",
    peer_competition: str = "pass",
) -> tuple[dict, dict, dict, dict]:
    single_report = {
        "status": single,
        "domains": [{"domain": 1, "status": single}],
    }
    peer_report = {"admission": {"result": peer}}
    peer_competition_report = {
        "status": peer_competition,
        "domains": [{"domain": 3, "status": peer_competition}],
    }
    oracle = {
        "comparison": {
            "classification": "inconclusive-candidate-bank-misses-success",
            "future_occupancy_discriminates_failure": False,
            "oracle_scope": {
                "label_generation_permitted": False,
                "runtime_input_permitted": False,
            },
        }
    }
    return single_report, peer_report, peer_competition_report, oracle


def test_readiness_is_single_only_when_peer_gate_failed(tmp_path) -> None:
    raw = tmp_path / "raw.npy"
    spatial = tmp_path / "spatial.npy"
    raw_sha = write_artifact(raw, b"raw")
    spatial_sha = write_artifact(spatial, b"spatial")
    single, peer, peer_competition, oracle = reports()

    result = audit_submission_readiness(
        raw_checkpoint=raw,
        expected_raw_sha256=raw_sha,
        spatial_adapter=spatial,
        expected_spatial_sha256=spatial_sha,
        single_competition=single,
        peer_motion=peer,
        peer_competition=peer_competition,
        future_oracle=oracle,
    )

    assert result["classification"] == "single-vehicle-candidate-only"
    assert result["artifact_identity_pass"]
    assert result["single_vehicle_gate_pass"]
    assert not result["mixed_peer_gate_pass"]
    assert not result["mixed_peer_motion_gate_pass"]
    assert result["mixed_peer_competition_gate_pass"]


def test_readiness_rejects_artifact_identity_mismatch(tmp_path) -> None:
    raw = tmp_path / "raw.npy"
    spatial = tmp_path / "spatial.npy"
    write_artifact(raw, b"raw")
    spatial_sha = write_artifact(spatial, b"spatial")
    single, peer, peer_competition, oracle = reports(peer="pass")

    result = audit_submission_readiness(
        raw_checkpoint=raw,
        expected_raw_sha256="0" * 64,
        spatial_adapter=spatial,
        expected_spatial_sha256=spatial_sha,
        single_competition=single,
        peer_motion=peer,
        peer_competition=peer_competition,
        future_oracle=oracle,
    )

    assert result["classification"] == "reject"
    assert not result["artifact_identity_pass"]


def test_readiness_accepts_multivehicle_only_after_peer_pass(tmp_path) -> None:
    raw = tmp_path / "raw.npy"
    spatial = tmp_path / "spatial.npy"
    raw_sha = write_artifact(raw, b"raw")
    spatial_sha = write_artifact(spatial, b"spatial")
    single, peer, peer_competition, oracle = reports(peer="pass")

    result = audit_submission_readiness(
        raw_checkpoint=raw,
        expected_raw_sha256=raw_sha,
        spatial_adapter=spatial,
        expected_spatial_sha256=spatial_sha,
        single_competition=single,
        peer_motion=peer,
        peer_competition=peer_competition,
        future_oracle=oracle,
    )

    assert result["classification"] == "multi-vehicle-candidate"
    assert result["mixed_peer_motion_gate_pass"]
    assert result["mixed_peer_competition_gate_pass"]


def test_motion_pass_cannot_hide_failed_peer_competition(tmp_path) -> None:
    raw = tmp_path / "raw.npy"
    spatial = tmp_path / "spatial.npy"
    raw_sha = write_artifact(raw, b"raw")
    spatial_sha = write_artifact(spatial, b"spatial")
    single, peer, peer_competition, oracle = reports(
        peer="pass", peer_competition="fail"
    )

    result = audit_submission_readiness(
        raw_checkpoint=raw,
        expected_raw_sha256=raw_sha,
        spatial_adapter=spatial,
        expected_spatial_sha256=spatial_sha,
        single_competition=single,
        peer_motion=peer,
        peer_competition=peer_competition,
        future_oracle=oracle,
    )

    assert result["classification"] == "single-vehicle-candidate-only"
    assert result["mixed_peer_motion_gate_pass"]
    assert not result["mixed_peer_competition_gate_pass"]
    assert "mixed-peer competition Gate failed" in result["reasons"]
