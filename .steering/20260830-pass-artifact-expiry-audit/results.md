# Results: Pass artifact-expiry audit

## Evidence boundary

- Baseline: `3fca7d4393bb63fb676547fb09b248d93a83bd55`
- AWSIM run: `output/20260830-162637`, Domain 1
- Last normal command before the visible failure: decision 5697
- First visible authority loss: decision 5698
- Replay-ready input:
  `000000003800-9e2c6962173873f9-pass-side-positive-successive-linearization-solve-rejected`
- Evidence class: simulation log plus immutable offline replay

Generated `output/`, architecture snapshots outside `.steering`, and result
JSON files remain untracked evidence and are not part of this Slice.

## Observed phenomenon

Episode 3 entered Pass on side `-1`, then published a certified stateless
sibling Bundle and committed the first cross-side replacement to side `+1` at
timestamp `1788074933.006760562`. The positive-side artifact remained normal
authority through decision 5697.

At decision 5683 a fresh immutable epoch produced:

- selected positive branch: rejected;
- negative sibling branch: certified;
- selected solve failure: dynamic-obstacle lateral row, stage 18;
- violation/tolerance: `0.000682/0.003026 m`;
- existing positive artifact: still executable.

At timestamp `1788074943.724917350` artifact 4577 exhausted. Decision 5698
therefore had no normal Pass authority and published canonical Emergency Stop.
The vehicle was still in Pass, the locked target was about 8 m ahead and the
first cross-side replacement had already latched the no-return state.

## Authority graph and causal chain

```text
current V2X/world
  -> two current-world side populations
  -> positive selected branch rejected
  -> negative sibling certified and banked
  -> persistent selected homotopy remains positive
  -> previously published positive artifact continues
  -> artifact cursor exhausts
  -> no selected-side certified replacement
  -> sibling adoption rejected by committed/no-return lifecycle
  -> normal authority unavailable
  -> Emergency Stop
  -> longitudinal-progress stall
  -> Recovery
```

The Emergency Stop and Recovery are downstream safety behavior. Extending the
artifact age, changing solver tolerance or adding a second resume rule would
mask the producer boundary.

## Same-snapshot architecture comparison

The default A--D/C--F observation-only comparison and the fixed/proof-guided
three-SQP comparisons retained the same world, physical model, seven-state
formulation and exact proof chain.

| Side | A persistent | B stateless direct | C rough lattice | D offline continuation | fixed/proof-guided SQP |
|---|---|---|---|---|---|
| selected `+1` | rejected | rejected | rejected | rejected | rejected |
| sibling `-1` | n/a | rejected | 41 certified | 2 certified | bounded production set rejected on this earlier snapshot |

Additional independently generated negative-side candidates were certified:

- diagonal: 40;
- physical diagonal: 16.

Typical certified negative-side results reached about `12.52 m` terminal
progress, `7.49 m/s` terminal speed and `0.484 m` minimum lateral reserve.
The unchanged exact wall, dynamic-obstacle and terminal-successor gates formed
complete ManeuverBundles.

The later live epoch at decision 5683 is stronger lifecycle evidence than the
earlier replay: the production sibling itself was certified, but could not be
adopted after the first committed side replacement.

## Root-cause classification

This failure requires a minimal causal cut set; forcing it into one leaf would
lose evidence.

1. **Selected-side continuation is unresolved, not proven physically
   infeasible.** All bounded selected-side arms failed, including D and three
   SQP passes. There is no bounded physical infeasibility certificate, so the
   correct classification is `Unknown`, not physical infeasibility.
2. **The visible Stop is a persistent lifecycle defect.** The world changed
   over a Pass lasting more than 10 s, well beyond the 2.4 s receding horizon.
   After one sibling publication the lifecycle permanently fixed the new
   side. When that side later became unsolved, a certified current-world
   sibling existed but the persistent Mission had no normal contingency owner
   between “keep selected Pass” and Emergency Stop.
3. **The frozen replay also exposes bounded candidate coverage.** A/B failed
   while independently generated C/D negative-side schedules succeeded. The
   three-member production population does not span those schedules for that
   earlier world.

Thus global physical infeasibility, wall-clearance tuning and solver tolerance
are falsified as explanations of the scene. The root architecture issue is a
long-lived Pass homotopy without an executable stateless contingency successor;
candidate coverage is a contributing cause.

## Existing patches and masks

- `Published stateless sibling Bundle adoption` correctly repairs the first
  pre-no-return selected-side loss, but permanently latches
  `mission_cross_side_transition_committed` and no-return.
- The certified sibling bank preserves evidence but cannot own a second normal
  transition.
- Published-artifact retention masks fresh selected-side failures until the
  cursor exhausts; it does not cause them.
- Emergency Stop is the correct final safety response and must remain.

## Next bounded implementation Slice

Do not add a second sibling retry or relax no-return. The next Slice must make
the already-certified terminal Stop suffix of the last actually published
ManeuverBundle an executable, same-formulation normal successor when its
normal prefix exhausts and no fresh selected-side Bundle joins.

The Slice must:

- use the immutable published artifact and its exact Stop trajectory;
- require the existing serialized-command join, current-world wall/dynamic
  revalidation and terminal certificate;
- preserve encounter, selected homotopy and no-return state;
- hand off once, atomically, through the existing canonical publisher;
- delete the direct `cursor exhausted -> immediate Emergency` edge when the
  certified successor is available;
- retain Emergency Stop when the successor is missing, stale or fails proof;
- add no lease, grace, timeout, retry, tolerance, clearance or config.

This successor is a bounded deceleration authority, not another pass-side
Mission rule. A later Slice may decide whether a stopped/slow safe state can
begin a new current-world encounter; that is explicitly out of scope here.

## Validation performed

- Default offline architecture comparison: complete, accepted candidates
  found only on the negative side.
- Fixed three-SQP comparison: neither selected positive nor bounded production
  negative set certified for the frozen input.
- Proof-guided comparison: no selected positive Bundle; baseline failed before
  proof-guided post-solve refinement could apply.
- Production code/config/authority: unchanged.

## Next dynamic acceptance

- Reproduce a Pass artifact exhaustion.
- A valid published Stop successor must publish without an Emergency authority
  hole and without changing side.
- Its solution, exact trajectory, physical certificate and command must share
  one immutable identity.
- Missing or failed successor proof must still select Emergency Stop.
- No second cross-side adoption, new lease or retained-age acceptance may
  appear.
