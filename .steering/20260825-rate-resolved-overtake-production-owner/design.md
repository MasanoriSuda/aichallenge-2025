# Design

## Root cause

The five-state Overtake formulation stores curvature as one control value per
coarse prediction stage. Publication consumes a stage endpoint as a steering
angle, then separately checks whether that angle is reachable in one 25 ms
control period. A plan can therefore be solver-feasible but publication-time
unreachable.

The six-state formulation removes this split ownership. Steering angle is a
state, steering rate is the lateral input, and publication samples the same
certified rate sequence at the 40 Hz boundary.

## Authority change

Use one rate-resolved normal pipeline for Track, Cruise, ShiftOut, Pass, and
Return:

1. build the semantic six-state request from the current `MpcProblem`;
2. revalidate the retained physically certified six-state plan against the
   current wall and dynamic-obstacle world;
3. publish only the resulting canonical authority or explicit Emergency;
4. bind the next worker snapshot to the steering command committed in step 3;
5. submit that immutable next problem asynchronously.

The tactical Mission, side, target, DP corridor, no-return and rear-clear logic
remain problem inputs. They do not publish commands.

## Deletion boundary

The production dispatch for ShiftOut/Pass/Return no longer calls
`canonical_overtake_production_control()` or its five-state async selector.
The old producer may not remain a reconnectable normal authority. Its dead
transport and telemetry are removed after the new path passes deterministic
and dynamic gates; no compatibility flag is introduced.

## Transition behavior

An intent transition with no current six-state proof fails closed to Emergency.
This Slice does not invent an unproved hold. If a first-cycle admission gap is
observed, it is handled as a separate atomic-admission defect using the same
six-state producer, not by restoring five-state authority.

The admission trigger is the canonical intent transition itself. It must not
depend on the rejection reason of a retained artifact from the preceding
intent: after a long Follow interval the old Cruise artifact can legitimately
be exhausted, and that artifact is not evidence for or against a new ShiftOut
problem. The transition path synchronously runs the ordinary six-state solver
and immutable physical-wall proof, installs only the certified artifact, then
runs the normal current-world retained join before publication.

## Canonical Overtake identity

The execution identity is resolved once before building the authority request
and MPC problem. A valid OvertakeLine mission owns the identity. Otherwise, a
validated DynamicEscape path contributes its target, attempt generation, side,
and ShiftOut intent. A malformed escape fails closed. This prevents the trace
from claiming ShiftOut while the solved problem still carries Idle, side zero,
and no progress-execution context.

The set of intents owned by the rate-resolved artifact is also defined once in
`mpcc_rate_resolved_execution_artifact`; production adaptation and retained
current-world revalidation consume that same definition. Local Track/Cruise
copies are prohibited by source-contract tests.
