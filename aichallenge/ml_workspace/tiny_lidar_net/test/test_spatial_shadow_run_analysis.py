import pytest

from analyze_spatial_shadow_run import (
    parse_runtime_config,
    parse_status_lines,
    summarize_intervals,
)


def test_shadow_status_parser_aggregates_coverage_and_runtime():
    log = """
[node] Core initialized. SpatialShadowConfig: hidden=128,projection=128,use_speed=1,max_speed_mps=12.000000,max_delta_rad=1.200000,speed_timeout_sec=0.100000,authority_enabled=1,authority_max_delta_rad=0.120000
[node] E2E_STATUS scans=100 stale=0 scan_hz=20.00 avg_inference_ms=5.00 max_inference_ms=12.00 inference_capacity_hz=200.00 spatial_shadow=99/100 shadow_skipped=1 shadow_errors=0 shadow_mean_abs_rad=0.01000 shadow_p95_abs_rad=0.10000 shadow_last_rad=-0.02000 shadow_prob_lnr=0.600,0.300,0.100 shadow_status=ok spatial_authority_enabled=1 spatial_authority_applied=99/100 spatial_authority_clipped=2 spatial_authority_mean_abs_rad=0.00900 spatial_authority_max_abs_rad=0.12000
[node] E2E_STATUS scans=200 stale=0 scan_hz=19.90 avg_inference_ms=7.00 max_inference_ms=20.00 inference_capacity_hz=142.86 spatial_shadow=100/100 shadow_skipped=0 shadow_errors=0 shadow_mean_abs_rad=0.03000 shadow_p95_abs_rad=0.20000 shadow_last_rad=0.04000 shadow_prob_lnr=0.100,0.200,0.700 shadow_status=ok spatial_authority_enabled=1 spatial_authority_applied=100/100 spatial_authority_clipped=3 spatial_authority_mean_abs_rad=0.02000 spatial_authority_max_abs_rad=0.12000
"""

    intervals = parse_status_lines(log)
    summary = summarize_intervals(intervals)

    assert len(intervals) == 2
    assert summary["coverage_fraction"] == pytest.approx(199.0 / 200.0)
    assert summary["skipped_count"] == 1
    assert summary["error_count"] == 0
    assert summary["min_scan_hz"] == pytest.approx(19.9)
    assert summary["weighted_avg_inference_ms"] == pytest.approx(6.0)
    assert summary["max_inference_ms"] == pytest.approx(20.0)
    assert summary["nonzero_interval_count"] == 2
    assert summary["authority_applied_count"] == 199
    assert summary["authority_clipped_count"] == 5
    assert summary["authority_max_abs_correction_rad"] == pytest.approx(0.12)
    assert parse_runtime_config(log) == {
        "hidden_dim": 128,
        "projection_dim": 128,
        "use_speed": True,
        "max_speed_mps": 12.0,
        "max_abs_delta_rad": 1.2,
        "speed_timeout_sec": 0.1,
        "authority_enabled": True,
        "authority_max_abs_delta_rad": 0.12,
    }


def test_shadow_status_parser_rejects_nonfinite_metrics():
    log = """
[node] E2E_STATUS scans=100 stale=0 scan_hz=20.00 avg_inference_ms=nan max_inference_ms=12.00 inference_capacity_hz=200.00 spatial_shadow=100/100 shadow_skipped=0 shadow_errors=0 shadow_mean_abs_rad=0.01000 shadow_p95_abs_rad=0.10000 shadow_last_rad=0.00000 shadow_prob_lnr=0.1,0.8,0.1 shadow_status=ok
"""
    with pytest.raises(ValueError, match="non-finite"):
        parse_status_lines(log)
