"""Physical body calibration must enclose the independently measured mesh."""
import json
import math
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('configuration', ['config.yaml', 'config_for_cloud.yaml'])
def test_nominal_footprint_contains_measured_solid_body(configuration):
    fixture = json.loads((ROOT / 'test/fixtures/awsim_body_projection.json').read_text())
    config = yaml.safe_load((ROOT / 'config' / configuration).read_text())
    footprint = config['stuck_recovery']['footprint']
    assert fixture['frame'] == 'base_link' and fixture['units'] == 'm'
    outside = [
        (forward, left) for forward, left in fixture['projection_hull_xy_m']
        if not (-footprint['rear_extent_m'] <= forward <= footprint['front_extent_m']
                and -footprint['right_extent_m'] <= left <= footprint['left_extent_m'])
    ]
    assert not outside, f'{configuration}: solid body vertices outside nominal footprint: {outside}'


@pytest.mark.parametrize('configuration', ['config.yaml', 'config_for_cloud.yaml'])
def test_combined_lateral_distance_does_not_shrink_peer_when_ego_is_calibrated(configuration):
    fixture = json.loads((ROOT / 'test/fixtures/awsim_body_projection.json').read_text())
    config = yaml.safe_load((ROOT / 'config' / configuration).read_text())
    footprint = config['stuck_recovery']['footprint']
    ego_extent = max(footprint['left_extent_m'], footprint['right_extent_m'])
    measured_peer_extent = max(abs(left) for _, left in fixture['projection_hull_xy_m'])
    # Planner lateral spacing is distinct from the full physical peer envelope.
    assert config['mpc']['v2x_vehicle_radius'] - ego_extent >= measured_peer_extent


@pytest.mark.parametrize('configuration', ['config.yaml', 'config_for_cloud.yaml'])
def test_nominal_peer_sphere_contains_each_body_about_observed_gnss_point(configuration):
    fixture = json.loads((ROOT / 'test/fixtures/awsim_peer_body_envelope.json').read_text())
    config = yaml.safe_load((ROOT / 'config' / configuration).read_text())
    nominal_radius = config['mpc']['v2x_peer_body_radius_m']
    assert fixture['frame'] == 'base_link' and fixture['v2x_position_frame'] == 'gnss_antenna'
    assert len(fixture['vehicles']) == 4
    assert 'vehicle_radius_m' not in config['stuck_recovery']['rear_safety']
    for vehicle in fixture['vehicles']:
        antenna = vehicle['antenna_base_link']
        radius = max(math.dist(vertex, antenna)
                     for vertex in fixture['convex_hull_vertices_base_link_xyz_m'])
        assert radius == pytest.approx(vehicle['nominal_3d_radius_m'], rel=0.0, abs=1e-12)
        assert nominal_radius >= radius, (configuration, vehicle['vehicle'], radius)
