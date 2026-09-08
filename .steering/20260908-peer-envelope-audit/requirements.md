# Physical peer envelope audit

The current physical circle is derived from combined lateral separation
minus ego half width. Correcting both widths still yields peer radius0.768m.
This accounts for lateral width, not the entire rigid body about the actual
V2X reference point. Confirm the producer origin and independent hull before
changing any physical certificate model.

Local Assembly-CSharp IL: GnssSensor.Start caches its own transform;
FixedUpdate converts that world position and adds MGRS offset. The Navsatfix
async publisher broadcasts this same MgrsPosition. Fanout.FillArrayMsg assigns
its XYZ directly. Scene Mono1406/GnssSensor and1375/NavsatfixRos2Publisher are
both on NavsatfixSensor under gnss_link, base_link forward offset-0.26m.
Controller tracking currently stores that XYZ directly. V2X has no yaw.

Rigid-body sphere containment about the antenna is independent of unknown
orientation: rotation preserves Euclidean norm. For the measured solid body,
maximum3D radius1.875424947m, outward-rounded1.876m. A planar half-width circle
cannot substitute for this. Position/prediction uncertainty remains separate.

This audit does not authorize changing V2X wire fields or taking heading from
an unobserved source. Compare the existing circle, a complete measured sphere
projection, and orientation-informed shapes with their required observations.
A corrected producer must preserve immutable world fingerprints and the
single normal authority. No ad hoc margin or target-only mask repair.
