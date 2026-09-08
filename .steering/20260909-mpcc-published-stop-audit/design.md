# Published Stop authority-loss observation

Continue the authorised MPCC completion work from the side-peer/Stop commit
f3b0338c. Integrated acceptance is open. This slice changes observation only.

At `20260909-side-peer-stop-dev2-r1`, Domain 2 decision 976 adopts certified
Stop source 442 / source decision 972. At decision 978 its current-world wall
proof rejects; the published command becomes external Emergency. The final
authority-loss recorder skips the event because `published_stop_retained` is
true, although this flag selects `canonical_normal_emergency_stop`, not a proved
retained artifact. The actual Stop source/artifact/publication clock is missing.
The saved pre-Stop source 976 and previously executed normal 962 cannot replace it.

Invariant: a failed current-world normal/Stop proof must be observable before
Emergency clears the execution ledger. Only an actual current certified authority
can suppress this capture. Keep requested Unknown/Stop-without-normal-source
exclusions, existing bounded async recording, and the exact snapshot/artifact
identity checks. Remove the false publication-label gate; do not change planner,
solver, candidates, wall/peer checks, command selection or runtime parameters.

First compile the actual observation guard in a native harness to reproduce the
missed Cruise-origin Stop loss and unaffected valid/unsupported cases. Then patch,
run focused/source/package checks and full build. Run one bounded diagnostic dev2
with the same control settings to obtain the missing tuple; any success is only
observation evidence. Stop at the first moving override with capture time or at
the existing six-lap/host deadline. Preserve and restore user JSON bytes.

No simultaneous simulation and production edit/build/replay. Compare the new
immutable source, actual executed controls, current-world state, same-run MCAP
and an independent Cartesian rollout before any physical-model repair. The
earlier affine-infeasible normal branch remains a separate candidate-generation
problem. No iteration, tolerance, margin, delay or fallback tuning.

Rollback: f3b0338c. Delete/revise this observer if the execution ledger or snapshot
owner is replaced; it has no authority API and no promotion to normal control.
