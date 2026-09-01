import numpy as np

from audit_spatial_candidate_replay import (
    clean_and_resize_ranges,
    load_candidate,
    longest_true_samples,
    summarize_prediction,
    summarize_teacher_alignment,
)


def test_load_candidate_uses_explicit_training_residual_scale(tmp_path):
    from lib.spatial_adapter import FrozenTinyLidarSpatialResidual

    source = FrozenTinyLidarSpatialResidual(
        input_dim=750,
        hidden_dim=128,
        max_abs_delta_rad=0.12,
        use_speed=True,
        use_base_steering=True,
        spatial_normalization="fixed_train_statistics",
        projection_dim=128,
    )
    checkpoint = tmp_path / "candidate.npy"
    np.save(
        checkpoint,
        {
            key.replace(".", "_"): value.detach().cpu().numpy()
            for key, value in source.state_dict().items()
        },
    )

    loaded = load_candidate(
        checkpoint,
        use_base_steering=True,
        max_abs_delta_rad=0.12,
    )

    assert loaded.max_abs_delta_rad == 0.12


def test_clean_and_resize_ranges_matches_production_contract():
    values = clean_and_resize_ranges(
        np.asarray([np.nan, np.inf, -1.0, 45.0], dtype=np.float32),
        input_dim=4,
        max_range_m=30.0,
    )
    np.testing.assert_allclose(values, [0.0, 30.0, 0.0, 30.0])


def test_longest_true_samples_does_not_join_separate_runs():
    assert longest_true_samples([True, True, False, True]) == 2


def test_prediction_summary_reports_authority_saturation():
    residual = np.asarray([0.0, 0.119, 0.14, -0.03], dtype=np.float32)
    probabilities = np.asarray(
        [[0.1, 0.8, 0.1], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]],
        dtype=np.float32,
    )
    report = summarize_prediction(
        residual,
        probabilities,
        np.ones(4, dtype=bool),
        authority_bound_rad=0.12,
    )
    assert report["authority_saturation_fraction"] == 0.25
    assert report["longest_authority_saturation_samples"] == 1
    assert report["near_authority_bound_fraction"] == 0.5
    assert report["longest_near_authority_bound_samples"] == 2
    assert np.isclose(report["near_authority_bound_threshold_rad"], 0.114)
    assert report["right_fraction"] == 0.5


def test_prediction_summary_rejects_invalid_near_bound_fraction():
    with np.testing.assert_raises_regex(ValueError, "near_bound_fraction"):
        summarize_prediction(
            np.asarray([0.0], dtype=np.float32),
            np.asarray([[0.0, 1.0, 0.0]], dtype=np.float32),
            np.ones(1, dtype=bool),
            authority_bound_rad=0.12,
            near_bound_fraction=0.0,
        )


def test_teacher_alignment_separates_material_sign_failure():
    report = summarize_teacher_alignment(
        np.asarray([0.0, 0.1, 0.1, -0.2], dtype=np.float32),
        np.asarray([0.0, 0.2, -0.2, -0.3], dtype=np.float32),
        np.ones(4, dtype=bool),
        material_delta_rad=0.02,
    )
    assert report["material_samples"] == 3
    assert np.isclose(report["material_sign_accuracy"], 2.0 / 3.0)
    assert np.isclose(report["material_opposite_sign_fraction"], 1.0 / 3.0)
