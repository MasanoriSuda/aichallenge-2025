# Geometry producer

Controller configuration builds recovery_footprint_ once and supplies it to
the normal physical wall world. A lateral tracking reserve is added downstream
by the existing wall-proof builder; nominal body dimensions must be correct
before that expansion. This slice replaces nominal estimates with measured
outward bounds, not an empirically larger tracking margin.

The fixture is independent of configuration literals: hull vertices come
from the enabled convex MeshCollider, converted through the actual hierarchy
into base_link axes. Rear0.51m remains larger than the measured0.378507m.
The common symmetric0.768m side bound contains both asymmetric mesh sides.

`resolve_peer_circle_radius` derives peer-only radius from the planner's
combined ego+peer lateral separation. With ego0.768 and old combined1.45,
the implied peer half width becomes0.682m. A second failing regression exposes
this. Use the two measured nominal half widths to set combined1.536m, leaving
the existing prediction/uncertainty margins separate. This repairs lateral
dimension accounting, not the full peer-circle longitudinal enclosure.

Validate current configs failing the enclosure check, then full package
tests/build and static/dynamic geometry regressions. A new race campaign must
seal the changed binary/config and cannot inherit old campaign passes.
Map coverage is still open; do not present this as complete world calibration.
