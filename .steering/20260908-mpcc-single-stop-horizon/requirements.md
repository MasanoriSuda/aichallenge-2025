# Single-vehicle Stop boundary

Run `20260908-mpcc-single-six-lap` finished six laps in266.276642s with one
wall penalty (summary8.210140s; event8.215042s). At decision11046 the current
world rejected both retained continuation and analytic track-reference Stop.
Stuck Recovery followed; certified Cruise resumed at11464.

The live independent optimized Stop worker is ShiftOut/Pass-only. Its offline
construction from source11046 accepts an impossible terminal velocity jump:
v0=8.209698148m/s, horizon1.792347522s, physical acceleration lower=-3m/s2.
The last QP acceleration is-26.7572m/s2. This is a separate producer defect,
not evidence that this unexecuted worker caused the live wall penalty.

First compare the fixed source, terminal tracking alternatives, and a
target-free offline horizon variant. Do not extend live worker scope or
change parameters, wall/peer proofs, authority or publisher behavior.
Changing time is a new candidate; retain the source identity and reseal its
candidate fingerprint. Reject retiming if any dynamic peer/tube is present.

Rollback baseline: a8b968ac2e4a86a28432a52d132774459d355cb5; preserve the
pre-audit controller and comparison binaries from run4.
