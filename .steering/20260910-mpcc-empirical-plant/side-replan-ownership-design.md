# Published encounter and early side replan

Baseline / rollback: `21b025f3`; authorised autonomous MPCC M1–M6 repair.

## Earliest invariant and causal reproduction

Run `20260910-nine-state-dev2-r14`, D2, decision1100 at1789043932.800897031:
source589 passes the canonical publisher and changes side−1 to+1 in generation1.
The publisher deliberately retires frozen Mission geometry. Tactical ownership
continues through immutable actual publication, not the presence of that geometry.
The three early-replan entry points instead use `!mission_path_frozen` and thus
reopen the uncommitted planner. At1789043933.065976830 the ordering guard sees
relative longitudinal6.95m/lateral+1.06m. At1112/1789043933.090959212 it emits
RecoverOccupiedPassSide (side_abort1; wall/contact/unknown/watchdog all0), while
the final command still certifies a complete Stop source604. No moving Emergency
is observed. D2shutdown begins1789043945.501578983. No exact1112solver snapshot
exists; source589 is an earlier immutable world, not a reconstructed1112world.

Producer: `mpc_controller_cpp.cpp` early ShiftOut side-replan applicability,
window, assessment and resolver dispatch. Downstream Recovery and retry block
react correctly to the wrong `side_abort`; they are not the producer to bypass.
`PublishedStatelessEncounterCannotReopenEarlyReplan` reproduces the old resolver
with published ownership and the logged ordering. The test fails against the
unchanged21b025f3library; its lateral-window variants are synthetic boundary
coverage, not invented exact1112 telemetry. Source wiring assertions join this
native policy to the actual publication predicate and all three entry points.

## Architecture comparison and decision

Historical ownership slices already retired Mission geometry and removed double
publication ledgers. Current bounded A/B/C/D uses source589 negative-side snapshot
with current libraries/toolr38, fixed coarse C/D schedules and unchanged limits:
all15candidates solver-reject. This is inconclusive/Unknown, not physical
infeasibility. The actual positive sibling589 was certified and published live;
the diagnostic uses different candidate generation and lacks that exact runtime
candidate/warm-start payload, so it cannot relabel the actual publication.

Changing the solver/backend or restoring frozen geometry does not repair a
consumer misclassifying an already published owner. The selected change makes
one pure applicability predicate recognise frozen Mission OR actual published
stateless source. Controller assessment/window/dispatch and core resolver share
it. Remove the three geometry-only applicability paths. Keep relative ordering
observable and retain its Abort/Switch policy for genuinely uncommitted contexts.
No timeout, margin, debounce, solver budget, new lease or command authority.
Physical wall/front-risk/target continuity and per-tick native/peer/Stop/serialized
packet certification remain independent. No second publication ledger.

## Acceptance and limits

Required: failing-before native ownership regression; frozen-owner and uncommitted
negative cases; source integration; full build/package; fixedcommit dev2. R14 is
failed regardless of certified Stop. Native policy is not integrated race proof.
Actual multi-tick Stop/rest/restart, repeated single/dev2, dev3/dev4, gates and
same-tar evaluation remain M4–M6. Append measured results only after execution.

Measured local validation: buildr32 passes26packages in4min15s; testsr27 passes
2410records/62groups with0errors/failures/skips in29.62s. R14all-emitted callback
p95/p99/max D1=12.362149/15.521216/26.403146ms, D2=14.782569/20.082579/27.403ms;
no adjacent overrun pairs. Source/receipt gaps remain measurements, not a race
acceptance claim. Monitor shutdown request was1789043945.0891376; the earlier
D2input failure528 is at rest and later1628 is after shutdown.
See side-replan-ownership-evidence.json. Fixedcommitdev2-r15 is next.
