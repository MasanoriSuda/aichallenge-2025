import analyze_recurrent_shadow_run as analyzer


def test_status_parser_preserves_shadow_and_authority_contract() -> None:
    text = (
        "E2E_STATUS scans=100 stale=0 scan_hz=20.00 "
        "avg_inference_ms=3.10 max_inference_ms=5.20 "
        "inference_capacity_hz=192.30 spatial_authority_enabled=1 "
        "recurrent_shadow=99/100 recurrent_skipped=1 recurrent_errors=0 "
        "recurrent_mean_abs_rad=0.031 recurrent_p95_abs_rad=0.081 "
        "recurrent_last_rad=0.020 recurrent_raw_last_rad=0.020 "
        "recurrent_hidden_norm=1.25 recurrent_resets=0 recurrent_status=ok"
    )

    parsed = analyzer.parse_status_lines(text)
    assert len(parsed) == 1
    assert parsed[0]["admitted"] == 99
    assert parsed[0]["spatial_authority_enabled"] is True
    assert analyzer.summarize(parsed)["coverage_fraction"] == 0.99


def test_runtime_config_parser_rejects_ambiguity() -> None:
    config = (
        "RecurrentShadowConfig: hidden=64,projection=128,use_speed=0,"
        "speed_embedding=16,max_speed_mps=12.000000,"
        "max_correction_rad=0.640000,deadband_rad=0.020000"
    )
    assert analyzer.parse_runtime_config(config)["correction_deadband_rad"] == 0.02

    other = config.replace("hidden=64", "hidden=32")
    try:
        analyzer.parse_runtime_config(config + "\n" + other)
    except ValueError as error:
        assert "ambiguous" in str(error)
    else:
        raise AssertionError("ambiguous runtime configuration was accepted")
