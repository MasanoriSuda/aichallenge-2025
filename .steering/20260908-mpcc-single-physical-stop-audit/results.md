# Sensor / collider / map boundary

The initial slowdown is strongly consistent with a physical wall contact.
No direct PhysX contact event was recorded, so distinguish reconstructed
geometry from an exact engine contact replay.

## Reconstruction

Read-only UnityPy1.25.3 extraction identifies active, enabled, convex, non-trigger
`GoKart1/Colliders/Collider`. Transform to serialized base_link gives body
forward[-0.378506,1.614851], left[-0.767159,0.767926], up[-0.073961,0.544225]m.
Current declared front1.49m plus margin0.05m does not enclose the actual front.
The contact below is on the right side, so the separate front undercoverage
is not established as its cause. Inactive LiDAR meshes are not counted.

Static collision mesh: level1 MeshCollider1157 / sharedassets1 Mesh108,
`citycircuit_marge_inner_10`. World transformation follows the actual hierarchy.
Environment1298 gives MGRS offset(89637.703125,43503.5,35.4000015).
Assembly-CSharp IL confirms Unity(x,y,z)→ROS(z,-x,y) and quaternion(-z,x,-y,w).
IMU publishes its actual transform.rotation with configured orientation noise0.
Its local Unity yaw-90 is ROS+90, undone by right multiplication of ROS-90.
GNSS altitude is MGRS Z directly. Recover antenna XY by undoing GNSSPoser's
planar0.26m lever arm, then apply actual body rotation to obtain base_link.

At source250.599994: reconstructed base_link
(89667.699199,43169.067950,42.501217), yaw0.218106.
No body/scene triangle intersection is found at samples250.35..250.60.
At250.649994/250.699994/250.749994, static triangles81055/81056 intersect the
3D convex body at roughly bodyX0.8/Y-0.71m, within the body's actual height.
Their maximum interior depths are3.50/4.73/3.78mm, comparable to the sensor's
MGRS float quantization. This supports contact but cannot resolve exact PhysX
skin/cooking or the first substep. Raw velocity collapses between250.60/250.635.

The reconstructed contact is near map(89668.655,43168.550), in pixel255/free,
0.599555m from the nearest occupied cell center and0.548311m from its boundary.
These distances use the native C++ center-origin convention; the earlier
0.555m center distance used a corner-origin convention and is superseded. Thus the map lacks the
physical barrier there, independently of sub-centimetre contact uncertainty.
The old map cannot certify complete actual wall coverage. Its provenance is
Lanelet2-derived occupancy, not the current simulator collision mesh.

## Localization contribution

At the same source250.599994, raw NavSatFix is covariance UNKNOWN/zeros.
GNSSPoser substitutes10m² and imu_gnss_poser maps that to100m². EKF has
globalY variance0.8171m² and its position is about0.50m above the GNSS estimate.
Do not treat covariance UNKNOWN as measured poor accuracy or as zero noise.

Two actual EKF binaries seeded with the same recorded pose/covariance at240s
receive the same pose/twist/clock. Only position measurement variance changes.
At250.605s, old100m² gives(89667.5740,43169.6645);0.1m² gives
(89667.7709,43169.1254). The second is much closer to reconstructed body pose.
Original hidden EKF history is absent; this comparison demonstrates the
measurement weighting contribution, not exact replay or closed-loop clearance.
Producer repair and validation are in `../20260908-gnss-unknown-covariance/`.

## Still open

- Produce and verify physical wall coverage for the current simulator assets;
  replacing a margin with a larger number is not evidence of full coverage.
- Correct measured body-envelope containment as its own model defect.
- Validate calibrated localization in closed-loop operation. Its covariance
  remains statistical, not a hard physical error bound.
- Capture actual contact events / useful app-window video if exact event proof
  is required. Existing primary-screen MP4 captures the desktop.
- Repeat fixed-binary race acceptance only after structural changes are tested.
  The two successful trials and the failed third trial remain unchanged.
