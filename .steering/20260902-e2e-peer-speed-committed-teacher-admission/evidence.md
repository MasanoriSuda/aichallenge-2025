# Evidence

## Runtime identity

Run: `output/20260902-e2e-peer-speed-committed-teacher`, evaluated on d3.

- d1/d2 remained the frozen MPC peers.
- d3 control mode: `speed_committed_teacher`.
- d3 maximum-forward-speed governor: `0.0`, as required by the previously
  certified teacher contract.
- acceleration: `0.8 m/s2`.
- packaged raw and spatial checkpoints loaded with spatial production
  authority disabled in teacher mode; recurrent model disabled.
- The same focused source/installed contract suites had just passed `95/95`.

Two isolated wheel-speed freshness misses at startup correctly published Stop.
They did not create the sustained failure below, but make the run ineligible
for a claim of perfect runtime admission.

## Closed-loop result: reject

The teacher crossed the packaged student's first stall time and initially
continued near `5.7--6.3 m/s`.  It demonstrated early side commitment, but
later entered a different unrecoverable contact state:

- distance: `743.719 m`;
- maximum speed: `6.523 m/s`;
- first sustained low speed: `160.682 s` after bag start;
- longest low-speed interval: `64.426 s`;
- positive-acceleration stall: `0.0 s` because the teacher had already selected
  hard brake;
- stall context: command `-1.0 m/s2`, steering `+0.64 rad`, front clearance
  `0.409 m`, right clearance `0.279 m`;
- the final motion Gate status is `fail`.

Immediately before the trap the 5-second telemetry moved from approximately
`6.04 m/s`, front `4.59 m`, `side-maintained` with side `-1/-1`, to speed zero,
front `0.48 m`, and `static-stop-envelope` with a newly visible side `0/1`.
The telemetry simultaneously reported a speed-dependent stop distance of
`21.27 m` and trigger distance of `25.80 m`, yet the side-maintained path kept
the lateral escape policy active until physical clearance collapsed.

This is not evidence that lowering a threshold would fix the student.  It is a
closed-loop falsification of the current teacher as a hard-label source for
this peer world: the teacher's speed-aware distance calculation and its
side-maintained longitudinal/lateral decision are not composed into a viable
trajectory at the observed speed.

## Decision

Reject the teacher run.  Do not create an outcome certificate, extract labels,
train a student or retry another seed.  Production checkpoints, launch defaults
and authority remain unchanged.

The next work must repair or replace the teacher policy at the decision level:
when predicted stopping distance greatly exceeds frontal clearance, an active
side commitment may continue only if the selected escape corridor has a
current physical viability certificate.  Otherwise the teacher must brake
before entering the no-return contact region.  That hypothesis requires a
separate offline replay and closed-loop teacher Slice; it must not be added as
a student runtime fallback.
