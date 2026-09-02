import hashlib

from audit_e2e_submission_readiness import audit_submission_readiness


RAW_RUNTIME_PATH = "/install/tinylidarnet_weights.npy"
SPATIAL_RUNTIME_PATH = "/install/spatial_steering_adapter.npy"
SINGLE_COMPETITION_SHA = "1" * 64
PEER_COMPETITION_SHA = "2" * 64
PEER_MOTION_SHA = "3" * 64


def write_artifact(path, value: bytes) -> str:
    path.write_bytes(value)
    return hashlib.sha256(value).hexdigest()


def runtime_contract(
    raw_sha: str,
    spatial_sha: str,
    raw_runtime_path: str = RAW_RUNTIME_PATH,
    spatial_runtime_path: str = SPATIAL_RUNTIME_PATH,
) -> dict:
    return {
        "control_mode": "fixed_lidar_brake",
        "checkpoint_path": raw_runtime_path,
        "checkpoint_sha256": raw_sha,
        "acceleration_mps2": 0.8,
        "maximum_forward_speed_mps": 4.6,
        "residual_checkpoint_path": "",
        "spatial_checkpoint_path": spatial_runtime_path,
        "spatial_checkpoint_sha256": spatial_sha,
        "spatial_use_base_steering": True,
        "spatial_authority_enabled": True,
        "spatial_authority_max_abs_delta_rad": 1.2,
        "recurrent_checkpoint_path": "",
        "recurrent_authority_enabled": False,
    }


def domain_runtime(contract: dict) -> dict:
    return {
        "control_mode": contract["control_mode"],
        "checkpoint_path": contract["checkpoint_path"],
        "acceleration_mps2": contract["acceleration_mps2"],
        "maximum_forward_speed_mps": contract[
            "maximum_forward_speed_mps"
        ],
        "residual_checkpoint_path": contract["residual_checkpoint_path"],
        "spatial_checkpoint_path": contract["spatial_checkpoint_path"],
        "spatial_expected_sha256": contract["spatial_checkpoint_sha256"],
        "spatial_use_base_steering": contract[
            "spatial_use_base_steering"
        ],
        "spatial_authority_enabled": contract["spatial_authority_enabled"],
        "spatial_authority_max_abs_delta_rad": contract[
            "spatial_authority_max_abs_delta_rad"
        ],
        "recurrent_checkpoint_path": contract[
            "recurrent_checkpoint_path"
        ],
        "recurrent_authority_enabled": contract[
            "recurrent_authority_enabled"
        ],
    }


def competition_report(
    contract: dict,
    *,
    domain: int,
    status: str = "pass",
    motion_sha: str = "0" * 64,
) -> dict:
    return {
        "schema_version": 1,
        "status": status,
        "run_dir": f"/run/domain-{domain}",
        "thresholds": {"max_penalty_count": 0},
        "expected_runtime": dict(contract),
        "checkpoint_artifact": {"sha256": contract["checkpoint_sha256"]},
        "spatial_checkpoint_artifact": {
            "sha256": contract["spatial_checkpoint_sha256"]
        },
        "domains": [
            {
                "domain": domain,
                "status": status,
                "runtime": domain_runtime(contract),
                "artifacts": {"motion_sha256": motion_sha},
            }
        ],
    }


def spatial_report(
    spatial_sha: str,
    *,
    domain: int,
    competition_sha: str,
    status: str = "pass",
    coverage: float = 0.995,
) -> dict:
    return {
        "schema_version": 1,
        "status": status,
        "domain": domain,
        "run_dir": f"/run/domain-{domain}",
        "production_gate_status": "pass",
        "runtime_evidence": {
            "competition_report_sha256": competition_sha,
        },
        "shadow_checkpoint": {"sha256": spatial_sha},
        "runtime_config": {
            "use_base_steering": True,
            "authority_enabled": True,
            "max_abs_delta_rad": 1.2,
            "authority_max_abs_delta_rad": 1.2,
        },
        "shadow": {
            "coverage_fraction": coverage,
            "error_count": 0,
            "non_ok_interval_count": 0,
            "stale_interval_count": 0,
            "interval_count": 3,
            "authority_enabled_interval_count": 3,
            "authority_applied_count": 299,
        },
    }


def oracle_report() -> dict:
    return {
        "comparison": {
            "classification": "inconclusive-candidate-bank-misses-success",
            "future_occupancy_discriminates_failure": False,
            "oracle_scope": {
                "label_generation_permitted": False,
                "runtime_input_permitted": False,
            },
        }
    }


def setup_case(tmp_path, *, peer_motion: str = "fail") -> dict:
    raw = tmp_path / "install-raw.npy"
    source_raw = tmp_path / "source-raw.npy"
    spatial = tmp_path / "install-spatial.npy"
    source_spatial = tmp_path / "source-spatial.npy"
    raw_sha = write_artifact(raw, b"raw")
    write_artifact(source_raw, b"raw")
    spatial_sha = write_artifact(spatial, b"spatial")
    write_artifact(source_spatial, b"spatial")
    contract = runtime_contract(
        raw_sha,
        spatial_sha,
        raw_runtime_path=str(raw.absolute()),
        spatial_runtime_path=str(spatial.absolute()),
    )
    return {
        "raw_checkpoint": raw,
        "source_raw_checkpoint": source_raw,
        "expected_raw_sha256": raw_sha,
        "spatial_adapter": spatial,
        "source_spatial_adapter": source_spatial,
        "expected_spatial_sha256": spatial_sha,
        "expected_runtime": contract,
        "expected_spatial_max_abs_delta_rad": 1.2,
        "single_competition": competition_report(contract, domain=1),
        "single_competition_sha256": SINGLE_COMPETITION_SHA,
        "single_spatial": spatial_report(
            spatial_sha,
            domain=1,
            competition_sha=SINGLE_COMPETITION_SHA,
        ),
        "single_domain": 1,
        "peer_motion": {"admission": {"result": peer_motion}},
        "peer_motion_sha256": PEER_MOTION_SHA,
        "peer_competition": competition_report(
            contract, domain=3, motion_sha=PEER_MOTION_SHA
        ),
        "peer_competition_sha256": PEER_COMPETITION_SHA,
        "peer_spatial": spatial_report(
            spatial_sha,
            domain=3,
            competition_sha=PEER_COMPETITION_SHA,
        ),
        "peer_domain": 3,
        "future_oracle": oracle_report(),
    }


def test_readiness_is_single_only_when_peer_motion_gate_failed(tmp_path) -> None:
    result = audit_submission_readiness(**setup_case(tmp_path))

    assert result["classification"] == "single-vehicle-candidate-only"
    assert result["artifact_identity_pass"]
    assert result["single_vehicle_gate_pass"]
    assert not result["mixed_peer_gate_pass"]
    assert not result["mixed_peer_motion_gate_pass"]
    assert result["mixed_peer_competition_gate_pass"]
    assert result["mixed_peer_spatial_gate_pass"]


def test_readiness_rejects_runtime_artifact_identity_mismatch(tmp_path) -> None:
    case = setup_case(tmp_path, peer_motion="pass")
    case["expected_raw_sha256"] = "0" * 64

    result = audit_submission_readiness(**case)

    assert result["classification"] == "reject"
    assert not result["artifact_identity_pass"]


def test_readiness_rejects_source_install_identity_mismatch(tmp_path) -> None:
    case = setup_case(tmp_path, peer_motion="pass")
    case["source_raw_checkpoint"].write_bytes(b"different-source")

    result = audit_submission_readiness(**case)

    assert result["classification"] == "reject"
    assert not result["artifact_identity_pass"]


def test_runtime_artifact_must_use_expected_install_path(tmp_path) -> None:
    case = setup_case(tmp_path, peer_motion="pass")
    wrong_path = "/install/not-the-runtime-artifact.npy"
    case["expected_runtime"]["checkpoint_path"] = wrong_path
    for report_name in ("single_competition", "peer_competition"):
        case[report_name]["expected_runtime"]["checkpoint_path"] = wrong_path
        case[report_name]["domains"][0]["runtime"][
            "checkpoint_path"
        ] = wrong_path

    result = audit_submission_readiness(**case)

    assert result["classification"] == "reject"
    assert not result["artifact_identity_pass"]


def test_readiness_accepts_multivehicle_only_after_all_peer_gates(tmp_path) -> None:
    result = audit_submission_readiness(
        **setup_case(tmp_path, peer_motion="pass")
    )

    assert result["classification"] == "multi-vehicle-candidate"
    assert result["mixed_peer_motion_gate_pass"]
    assert result["mixed_peer_competition_gate_pass"]
    assert result["mixed_peer_spatial_gate_pass"]


def test_lax_competition_report_cannot_pass_readiness(tmp_path) -> None:
    case = setup_case(tmp_path, peer_motion="pass")
    case["single_competition"]["expected_runtime"].pop(
        "spatial_authority_enabled"
    )

    result = audit_submission_readiness(**case)

    assert result["classification"] == "reject"
    assert not result["single_vehicle_competition_gate_pass"]
    assert any(
        "expected runtime missing spatial_authority_enabled" in reason
        for reason in result["reasons"]
    )


def test_wrong_competition_domain_cannot_pass_readiness(tmp_path) -> None:
    case = setup_case(tmp_path, peer_motion="pass")
    case["single_competition"]["domains"][0]["domain"] = 2

    result = audit_submission_readiness(**case)

    assert result["classification"] == "reject"
    assert any("domain identity mismatch" in reason for reason in result["reasons"])


def test_motion_pass_cannot_hide_failed_peer_competition(tmp_path) -> None:
    case = setup_case(tmp_path, peer_motion="pass")
    case["peer_competition"]["status"] = "fail"
    case["peer_competition"]["domains"][0]["status"] = "fail"

    result = audit_submission_readiness(**case)

    assert result["classification"] == "single-vehicle-candidate-only"
    assert result["mixed_peer_motion_gate_pass"]
    assert not result["mixed_peer_competition_gate_pass"]


def test_peer_motion_must_match_competition_evidence(tmp_path) -> None:
    case = setup_case(tmp_path, peer_motion="pass")
    case["peer_competition"]["domains"][0]["artifacts"][
        "motion_sha256"
    ] = "f" * 64

    result = audit_submission_readiness(**case)

    assert result["classification"] == "single-vehicle-candidate-only"
    assert any(
        "motion evidence identity mismatch" in reason
        for reason in result["reasons"]
    )


def test_single_spatial_coverage_is_mandatory(tmp_path) -> None:
    case = setup_case(tmp_path, peer_motion="pass")
    case["single_spatial"]["shadow"]["coverage_fraction"] = 0.98

    result = audit_submission_readiness(**case)

    assert result["classification"] == "reject"
    assert not result["single_vehicle_spatial_gate_pass"]


def test_spatial_report_must_bind_exact_competition_report(tmp_path) -> None:
    case = setup_case(tmp_path, peer_motion="pass")
    case["single_spatial"]["runtime_evidence"][
        "competition_report_sha256"
    ] = "f" * 64

    result = audit_submission_readiness(**case)

    assert result["classification"] == "reject"
    assert any(
        "competition report identity mismatch" in reason
        for reason in result["reasons"]
    )


def test_missing_peer_spatial_prevents_multivehicle_promotion(tmp_path) -> None:
    case = setup_case(tmp_path, peer_motion="pass")
    case["peer_spatial"] = None

    result = audit_submission_readiness(**case)

    assert result["classification"] == "single-vehicle-candidate-only"
    assert not result["mixed_peer_spatial_gate_pass"]


def test_spatial_authority_must_have_been_applied(tmp_path) -> None:
    case = setup_case(tmp_path, peer_motion="pass")
    case["single_spatial"]["shadow"]["authority_applied_count"] = 0

    result = audit_submission_readiness(**case)

    assert result["classification"] == "reject"
    assert any(
        "authority was never applied" in reason for reason in result["reasons"]
    )
