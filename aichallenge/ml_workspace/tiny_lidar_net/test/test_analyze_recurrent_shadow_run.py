import analyze_recurrent_shadow_run as analyzer


def test_runtime_log_loader_selects_requested_domain(tmp_path) -> None:
    status = (
        "E2E_STATUS scans=1 stale=0 scan_hz=20.0 "
        "avg_inference_ms=1.0 max_inference_ms=2.0 "
        "inference_capacity_hz=500.0 recurrent_shadow=1/1 "
        "recurrent_skipped=0 recurrent_errors=0 "
        "recurrent_mean_abs_rad=0.01 recurrent_p95_abs_rad=0.02 "
        "recurrent_last_rad=0.01 recurrent_raw_last_rad=0.01 "
        "recurrent_hidden_norm=1.0 recurrent_resets=0 recurrent_status=ok"
    )
    for domain in (1, 3):
        domain_dir = tmp_path / f"d{domain}"
        domain_dir.mkdir()
        (domain_dir / "autoware.log").write_text(
            f"d{domain} {status}\n",
            encoding="utf-8",
        )

    text, paths, used_fallback = analyzer.load_runtime_log_text(tmp_path, 3)

    assert text.startswith("d3 ")
    assert paths == [str(tmp_path / "d3" / "autoware.log")]
    assert not used_fallback


def test_status_parser_preserves_shadow_and_authority_contract() -> None:
    text = (
        "E2E_STATUS scans=100 stale=0 scan_hz=20.00 "
        "avg_inference_ms=3.10 max_inference_ms=5.20 "
        "inference_capacity_hz=192.30 spatial_authority_enabled=1 "
        "recurrent_shadow=99/100 recurrent_skipped=1 recurrent_errors=0 "
        "recurrent_mean_abs_rad=0.031 recurrent_p95_abs_rad=0.081 "
        "recurrent_last_rad=0.020 recurrent_raw_last_rad=0.020 "
        "recurrent_hidden_norm=1.25 recurrent_resets=0 recurrent_status=ok"
        " recurrent_authority_enabled=1 recurrent_authority_applied=99/100"
        " recurrent_authority_clipped=2"
        " recurrent_authority_mean_abs_rad=0.030"
        " recurrent_authority_max_abs_rad=0.240"
    )

    parsed = analyzer.parse_status_lines(text)
    assert len(parsed) == 1
    assert parsed[0]["admitted"] == 99
    assert parsed[0]["spatial_authority_enabled"] is True
    assert parsed[0]["recurrent_authority_enabled"] is True
    assert parsed[0]["recurrent_authority_applied"] == 99
    assert parsed[0]["recurrent_authority_clipped"] == 2
    assert analyzer.summarize(parsed)["coverage_fraction"] == 0.99
    assert analyzer.summarize(parsed)["max_authority_abs_rad"] == 0.24


def test_runtime_config_parser_rejects_ambiguity() -> None:
    config = (
        "RecurrentShadowConfig: hidden=64,projection=128,use_speed=0,"
        "speed_embedding=16,max_speed_mps=12.000000,"
        "max_correction_rad=0.640000,deadband_rad=0.020000,"
        "authority_enabled=0,authority_max_correction_rad=0.240000"
    )
    assert analyzer.parse_runtime_config(config)["correction_deadband_rad"] == 0.02
    assert analyzer.parse_runtime_config(config)["authority_enabled"] is False

    other = config.replace("hidden=64", "hidden=32")
    try:
        analyzer.parse_runtime_config(config + "\n" + other)
    except ValueError as error:
        assert "ambiguous" in str(error)
    else:
        raise AssertionError("ambiguous runtime configuration was accepted")


def test_runtime_config_parser_accepts_frozen_shadow_only_log() -> None:
    config = (
        "RecurrentShadowConfig: hidden=64,projection=128,use_speed=0,"
        "speed_embedding=16,max_speed_mps=12.000000,"
        "max_correction_rad=0.640000,deadband_rad=0.020000"
    )

    parsed = analyzer.parse_runtime_config(config)
    assert parsed["authority_config_present"] is False
    assert parsed["authority_enabled"] is False
    assert parsed["authority_max_abs_correction_rad"] is None
