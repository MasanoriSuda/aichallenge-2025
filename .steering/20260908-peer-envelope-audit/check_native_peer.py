"""Sealed body-overlap witness against the actual native producer and predicate."""
from pathlib import Path
import json
import math
import subprocess
import yaml

package = Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
source = (package/'src/mpcc_rate_resolved_retained_revalidation.cpp').read_text()
start = source.index('std::optional<double> resolve_peer_circle_radius(')
end = source.index('std::optional<FollowTargetObservation>',start)
function = source[start:end]
config = yaml.safe_load((package/'config/config.yaml').read_text())
f = config['stuck_recovery']['footprint']
geometry = json.loads(Path('/output/20260908-mpcc-stop-reference-single-r3/unity-collider-geometry.json').read_text())
body = next(c for c in geometry['colliders'] if c['path']=='/GoKart1/Colliders/Collider')['vertices_base_link_xyz']
antenna = [-.25999999046325684, 0., 0.]
radius = max(math.sqrt(sum((p[i]-antenna[i])**2 for i in range(3))) for p in body)
root = Path('/output/20260908-peer-envelope-audit')
root.mkdir(exist_ok=True)
fixture = json.loads((package/'test/fixtures/awsim_body_projection.json').read_text())
hull = fixture['projection_hull_xy_m']
point = max(hull,key=lambda p:p[0])
# Same-heading solid bodies: peer GNSS at(0,0), ego base_link at(2.2,0).
# This measured peer projection vertex is inside the measured ego convex projection.
point_world = [point[0]-antenna[0], point[1]]
point_in_ego = [point_world[0]-2.2, point_world[1]]
products = [(b[0]-a[0])*(point_in_ego[1]-a[1])-(b[1]-a[1])*(point_in_ego[0]-a[0]) for a,b in zip(hull,hull[1:]+hull[:1])]
assert all(x>=-1e-12 for x in products) or all(x<=1e-12 for x in products)
report = dict(peer_gnss_world=[0.,0.],ego_base_link_world=[2.2,0.],yaw=0.,
              shared_measured_projection_point=point_world,ego_projection_contains_peer_projection_vertex=True,
              measured_3d_enclosure_radius=radius,rounded_nominal_radius=math.ceil(radius*1000)/1000,
              scope='Static2D shape counterexample; not a3Dengine contact or a full racing command/tactical guard bypass.')
(root/'overlap-witness.json').write_text(json.dumps(report,indent=2)+'\n')
code = """#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <multi_purpose_mpc_ros/recovery_footprint.hpp>
namespace recovery = multi_purpose_mpc_ros::recovery_footprint;
constexpr double kIdentityTolerance = 1e-9;
""" + function + f"""
int main() {{
  const recovery::FootprintExtents ego{{{f['front_extent_m']},{f['rear_extent_m']},{f['left_extent_m']},{f['right_extent_m']},{f['margin_m']}}};
  const auto radius = resolve_peer_circle_radius({config['mpc']['v2x_vehicle_radius']},ego,0.05);
  if (!radius) return 2;
  const auto clearance = recovery::circle_obstacle_clearance_at_time(
    ego, recovery::Pose2D{{2.2,0.,0.}},recovery::CircleObstacle{{0.,0.,0.,0.,radius.value()}},0.);
  if (!clearance) return 2;
  std::cout << "peer_radius=" << radius.value() << " clearance=" << clearance.value() << "\\n";
  return clearance.value() <= 0. ? 0 : 1;
}}
"""
(root/'native-peer.cpp').write_text(code)
subprocess.run(['g++','-std=c++17','-O0','-I'+str(package/'include'),str(root/'native-peer.cpp'),str(package/'src/recovery_footprint.cpp'),'-o',str(root/'native-peer')],check=True)
subprocess.run([str(root/'native-peer')],check=True)
