# Results

## Observed phenomenon

Dynamic run `output/20260830-043824` completed full Overtake phase chains,
including `Idle -> ShiftOut -> Pass -> Return -> Idle`.  It also reproduced a
normal-authority loss at decision 3149 while following `d2`:

- decision 3148 still retained the previously published normal solution;
- decision 3149 could not certify its exact maximum-braking terminal Stop
  successor against the current wall model;
- production published the external Emergency Stop instead of an uncertified
  normal trajectory;
- the observation-only recorder froze the current problem without changing
  authority.

The frozen artifact is:

`output/20260830-043824/d1/mpcc_architecture_snapshots/000000003149-09185c6f27255e42-follow-side-neutral-physical-proof-terminal-contingency-unavailable/snapshot.yaml`

Its failure boundary is `physical-proof / terminal-contingency-unavailable`.
The artifact owns the exact production Stop steering policy and the physical
minimum acceleration of `-3.0 m/s2`; it contains no rejected QP because the
failure occurred at the proof boundary after source construction.

## Causal chain

1. The persistent Follow/neutral artifact was retained across world updates.
2. Its ordinary next publisher interval remained executable at decision 3148.
3. At decision 3149 the current-world recursive Stop successor could no longer
   be constructed and wall-certified (`wall_reason=invalid_rollout`).
4. Because normal authority requires a recursively safe successor, production
   correctly rejected the retained artifact and published Emergency Stop.
5. Offline reconstruction from the exact same immutable world found a
   stateless right-side current-world trajectory whose normal suffix and exact
   Stop successor were both wall- and obstacle-certified.

The visible Emergency Stop is therefore a downstream safety response.  The
upstream defect is retaining tactical geometry whose current-world successor
set has become inferior to a newly reconstructed homotopy.

## Architecture comparison

The source fingerprint was `655375378250292802`.  Applicable arms for the
captured Follow intent produced:

| Arm | Result | Relevant evidence |
| --- | --- | --- |
| A: persistent Mission + seven-state SQP | rejected | solver rejected; dynamic effective-progress violation at stage 6; no certified bundle |
| B-left: stateless current-world + same SQP | rejected | solver rejected; effective-progress violation at stage 8; no certified bundle |
| B-right: stateless current-world + same SQP | accepted | solve 89.3891 ms; terminal progress 7.55724 m; terminal velocity 1.85849 m/s; minimum lateral reserve 0.804954 m; exact normal and Stop certificates present |

C/D/G are not valid arms for this Follow-intent snapshot and were not assigned
synthetic results.  The specified exit classification is unambiguous for the
arms which do apply:

**A fails and B succeeds: persistent Mission lifecycle defect.**

This is not evidence for changing clearance, braking, horizon, weights or
solver tolerance.  B used the same seven-state SQP and the same immutable
physical world; the material difference was rebuilding the trajectory from the
current world instead of retaining the persistent Mission geometry.

## Implemented audit changes

- Sealed the exact Stop path-tracking policy and braking limit in `ReplayWorld`
  and the interaction fingerprint.
- Added schema-v3 source-only proof-boundary snapshots while preserving v2
  loading without inventing present-day policy values.
- Added an observation-only terminal-proof recorder with no Store, publisher,
  Mission or authority edge.
- Required every accepted comparison bundle to own its exact terminal Stop
  trajectory plus wall and dynamic certificates.
- Made recorded QP data optional so proof-boundary snapshots remain comparable;
  exact-QP/external-primal arms reject explicitly when it is absent.
- Added source-contract and comparison tests for missing and accepted recursive
  Stop certificates.

No production admission, publisher or authority rule changed in this Slice.

## Validation

- `make autoware-build`: 25 packages succeeded.
- Focused architecture snapshot/comparison tests: 2/2 succeeded.
- Full `multi_purpose_mpc_ros` package test: 54/54 CTest cases succeeded;
  `2178 tests, 0 errors, 0 failures, 0 skipped`.
- Dynamic `make dev2`: observation captured; two complete Overtake phase chains
  were observed.  The run is evidence capture, not a race-quality acceptance
  run, because a later stuck-Recovery episode remained.
- Offline comparison: B-right accepted with exact recursive certificates while
  A failed on the identical fingerprint.

## Remaining production change

The next Slice should replace the obsolete geometry ownership edge, not add a
resume rule or tune a threshold:

- retain only encounter identity, selected homotopy, commit/no-return state and
  the last actually published certified artifact;
- rebuild path samples, corridor and phase-transition trajectory from the
  current world;
- admit a replacement only through the existing exact normal and terminal Stop
  certificates;
- remove the superseded persistent-geometry producer in the same Slice;
- keep the external Emergency supervisor unchanged.

Production must not simply choose B-right because this one replay succeeded.
The next change must first identify the exact producer/consumer edge which
retains old geometry and atomically replace that edge with current-world bundle
ownership.

## Next dynamic checks

- A previously failing Follow/neutral encounter must adopt a current-world
  certified homotopy before Emergency is required.
- No stale fingerprint, no no-return side reversal and no uncertified Stop
  successor may be published.
- `ShiftOut -> Pass -> Return -> Idle` must continue to complete.
- Callback p95/p99 and consecutive overruns must be recorded; the observation
  recorder itself caused one expected 41.239 ms callback during this audit and
  must remain disabled outside evidence capture.
