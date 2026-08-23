# Audit

## Root cause

The Overtake five-state solver, exact physical certificate and canonical
command chain are already complete. Production nevertheless converts the same
primal into a legacy-shaped vector and allows circuit/reentry failure to fall
through to three-state MPCC. The final trace consequently has no canonical
plan identity even when canonical shadow selected a valid current-world plan.

The resulting split is causal, not cosmetic: in
`output/20260824-072942`, canonical shadow retained 71 complete selections at
the required wall clearance, while the published old path later encountered a
wall rejection and changed phase. Candidate, certificate and command were not
one immutable authority object.

## Rejected production Gate

`output/20260824-074307`, Domain 1, decision 2410 proved that early return from
the legacy solver was necessary but insufficient:

1. canonical solution 2410 passed and reached final-command processing;
2. the active-overtake wall admission reconstructed yaw from the canonical
   x/y visualization prediction and measured a generic front clearance of
   0.00 m;
3. the planner/canonical contract reported 1.89 m corridor reserve and had
   already passed the exact pose/yaw 0.40 m lateral-footprint proof;
4. the wall hold changed speed and steering, invalidated Mission generation 1
   and emitted `legacy-normal-bypass`.

This was not a stricter repeat of the same proof. The canonical oracle uses
the solved five-state pose yaw and expands the footprint laterally by the wall
contract. The wall-admission monitor uses x/y tangent yaw and a generic
all-direction proximity distance. The first violated invariant is therefore
that a selected solution, executed trajectory and physical certificate must
share one immutable representation.

The Gate also falsified the synchronous latency bridge. The prior shadow run
reported about 4.3 ms async compute and 71 current-ready results. With the
callback solve present, async compute reached about 50 ms and callback solves
repeatedly exhausted 4000 iterations. The bridge was a competing solve owner
and a contributor to the missing-result condition it attempted to mask.

`output/20260824-080151`, Domain 1, then proved that the corrected ShiftOut
boundary worked but was incomplete at its state-transition edge:

1. ShiftOut published only certified five-state commands or explicit
   Emergency, without legacy wall reinterpretation;
2. runtime wall preplan invalidated the Mission and entered DynamicWait with a
   rolling ShiftOut prefix;
3. the prefix then disappeared while DynamicWait state remained active;
4. intent resolution correctly returned `unknown/dynamic-wait-without-lateral-authority`;
5. `get_control()` incorrectly treated that invalid result as permission to
   enter legacy normal control and emitted `legacy-normal-bypass` at decision
   2405.

The first violated invariant is again authority ownership: loss of an
executable canonical path is an explicit failure of the current authority, not
a grant of authority to a different formulation. The causal correction is to
return the existing canonical Emergency at that boundary. Recovery/Rejoin is
not promoted by this Slice and remains separately auditable.

## Rejected alternatives

- Copy canonical identity onto the converted command: hides dual formulation.
- Keep three-state as availability fallback: preserves the root cause.
- Add an entry grace/lease: makes stale time, not evidence, own authority.
- Tune wall or solver parameters: does not repair command ownership.
- Preserve callback synchronous solve: duplicates the async producer and
  violates the runtime budget.
- Convert the x/y wall monitor into another canonical proof: duplicates an
  oracle already executed by fresh and retained selection.
- Treat invalid DynamicWait as ordinary Cruise/Follow: changes authority after
  evidence loss and recreates the same legacy bypass.

## Intended causal repair

For Overtake intent, the async worker is the only five-state solve producer.
Select and publish its current-world-certified canonical object, otherwise
fail explicitly. Canonical output does not enter the legacy active-overtake
x/y wall admission. Because every Overtake cycle returns before the legacy
solver and every canonical command carries its own exact physical certificate,
no downstream normal owner can reinterpret or replace the decision.

## Accepted production evidence

`output/20260824-081312` exercised ShiftOut after the final boundary fix. All
21 emitted ShiftOut authority traces were either complete certified canonical
five-state commands (9) or explicit Emergency (12). No ShiftOut, Pass, Return
or unresolved DynamicWait trace used legacy normal authority, and the old wall
admission did not fire. The async worker returned to its non-contended range of
approximately 4--8 ms during ordinary ShiftOut.

The physical certificate later rejected an incoming solution containing a
hard wall contact, and the tactical state moved through DynamicWait to
Recovery. That is the intended separation: unsafe evidence was rejected
without changing to a competing normal formulation. Recovery/Rejoin and the
subsequent independent Track/Cruise 4000-iteration failure remain distinct
migration work; neither is hidden by this Slice.
