"""Read-only native-grid coverage and reference-path comparison.

The candidate projects a selected class of triangles, not a complete 3D
collision world. This script cannot certify simulator safety.
"""
from collections import Counter
from dataclasses import asdict
import csv
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image
from scipy.ndimage import label
import yaml

package = Path('aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
sys.path.insert(0, str(package / 'tools/kaleidoscope'))
from kaleidoscope import trajectory_clearance as tc

root = Path('output/20260908-mpcc-stop-reference-single-r3')
original_yaml = package / 'env/final_ver3/occupancy_grid_map.yaml'
config = yaml.safe_load(original_yaml.read_text())
config['image'] = 'diagnostic-wall-candidate.pgm'
candidate_yaml = root / 'diagnostic-wall-candidate.yaml'
candidate_yaml.write_text(yaml.safe_dump(config))
rows = list(csv.DictReader((package / 'env/final_ver3/traj_mincurv.csv').open()))
poses = tuple(tc.Pose2D(float(r['x_m']), float(r['y_m']), float(r['psi_rad']),
                        float(r['s_m']), float(r['kappa_radpm'])) for r in rows)
report = {'scope': __doc__, 'reference_pose_count': len(poses), 'maps': {}}
for name, path in [('original', original_yaml), ('candidate', candidate_yaml)]:
    grid = tc.load_occupancy_grid(path)
    raster = np.asarray(Image.open(grid.spec.image_path)) == 0
    components, count = label(raster, np.ones((3, 3)))
    sizes = np.bincount(components.ravel())[1:]
    item = {'component_sizes': sizes.tolist(), 'components': count,
            'contact_cell_state': grid.state_at_world(89668.65526570712, 43168.54996447149).value,
            'footprints': {}}
    for margin in [0.0, 0.05, 0.25]:
        vehicle = tc.VehicleFootprintSpec.from_extents(
            1.615, .51, .768, .768, margin_front_m=margin,
            margin_rear_m=margin, margin_left_m=margin, margin_right_m=margin)
        result = tc.validate_clearance(
            grid, poses, vehicle,
            options=tc.ValidationOptions(circular=True, include_sweep=True,
                                         calculate_clearance=False))
        item['footprints'][str(margin)] = {
            'is_safe': result.is_safe,
            'colliding_point_count': result.colliding_point_count,
            'colliding_segment_count': result.colliding_segment_count,
            'issue_counts': dict(Counter(i.code for i in result.issues)),
            'issues': [asdict(i) for i in result.issues],
        }
    report['maps'][name] = item
(root / 'diagnostic-wall-feasibility.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps({name: {**value, 'footprints': {
    margin: {k: v for k, v in data.items() if k != 'issues'}
    for margin, data in value['footprints'].items()}}
    for name, value in report['maps'].items()}, indent=2))
