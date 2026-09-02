# Design

The old teacher scores angular openings in one 180-degree LiDAR scan.  This
Slice adds an offline counterfactual layer that asks a different question:
would the rectangular kart fit through the complete candidate motion and
still have a straight braking suffix to zero speed?

For each current scan and speed, the audit samples a bounded set of steering
offsets around the actually published steering.  A kinematic bicycle rolls
each offset through:

1. a first lateral shift segment;
2. an equal-duration counter-steer segment; and
3. a zero-steer, `-1.0 m/s2` full-stop suffix.

LiDAR points are transformed from the known `x=1.65 m` sensor origin into the
rear-axle `base_link` frame.  At every rollout state, the point-to-oriented-
rectangle distance is compared with the frozen clearance margin.  A candidate
is feasible only if the complete swept footprint and terminal stop are clear.

The certificate deliberately remains weaker than a runtime dynamic-obstacle
certificate.  One scan treats points as static and cannot prove future peer
motion or occluded wall geometry.  Its role is to falsify the present gap
teacher cheaply:

- selected side infeasible and opposite side feasible: candidate/side defect;
- both sides infeasible: current scan cannot support an escape claim;
- both feasible but failed run still diverges: temporal/dynamic supervision is
  still missing.

No production authority changes in this Slice.

