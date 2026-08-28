# Design

## Initial hypotheses

1. **Intent handoff lifecycle defect**: a new intent invalidates the previous
   artifact by semantic identity before a certified current-intent artifact is
   available, leaving a normal-authority hole.
2. **Actuation-origin mismatch**: the solver and retained proof are initialized
   from different steering states, causing first-stage rate infeasibility or
   `steering-unreachable`, especially while the vehicle is stationary.
3. **Single-SQP / numerical failure**: the current physical problem is valid,
   but the live single solve cannot produce a certified artifact in time.
4. **Physical infeasibility**: the current pose and wall corridor genuinely
   make the requested Track/Cruise suffix impossible.

## Investigation order

1. Identify the first previously accepted authority that becomes unavailable,
   rather than starting from the later repeated emergency symptom.
2. Join the decision to the exact producer sequence and worker result.
3. Compare candidate and executed retained rejection reasons at the same world.
4. Re-run the captured QP with its exact warm start and inspect first violated
   physical constraint.
5. Only after classification, choose the smallest structural correction which
   restores a single causal observation/actuation/publication chain.

## Forbidden corrections

- retain the old intent for longer by time alone;
- relax steering rate, wall margin or solver tolerance;
- synchronously duplicate the asynchronous solve in the control callback;
- make emergency output a substitute for missing normal authority.

## First frozen boundary

The first sustained loss begins at d1 decision 999.  The last executed plan is
sequence 310; the certified store already contains newer candidate sequence
384.  The executed plan first fails progress continuity and then exhausts its
horizon.  The newer candidate is not adopted because its cursor-zero steering
command is unreachable from the last actually published command.

Worker telemetry shows that this is not simply a lack of solved QPs: in the
preceding two-second window 30 of 38 consumed worker results solved and the
certified store advanced from sequence 310 to 384.  Production still loses
authority because the asynchronously completed candidate is always treated as
an unexecuted stage-zero plan, while the publisher has continued executing a
different predecessor during the 80--165 ms solve delay.

This points to a **causal asynchronous join defect**.  Before changing
authority, an audit-only comparison sampled the same immutable candidate on
its source prediction clock and ran the unchanged current-world steering,
wall and obstacle proofs.  Its result was logged beside the production
cursor-zero rejection and could not publish a command.

## Falsification result

`output/20260829-001005/d1` disproved the narrow hypothesis that the consumer
merely forgot to advance the candidate cursor.  At the first failure the
normal candidate requested `0.087963 rad` after `0.125 s`, while the exact
serialized predecessor could only reach `[0.222855, 0.274468] rad`.  The
source-timed candidate therefore remained `steering-unreachable`.

The mismatch is produced while the worker solves.  The previous certified
plan continues to publish and turns toward `+0.248661 rad`; the new QP, sealed
from its earlier current-world observation, turns back toward `+0.087963 rad`.
No command from the new artifact has crossed the wire, so advancing its clock
would falsely claim that its negative steering-rate prefix was executed.

This classifies the blocker as an **asynchronous publication scheduling /
successor-join defect**, not wall infeasibility and not a missing solved QP.
It also explains why loosening current-world reachability would be unsafe.
The temporary source-time audit was removed after falsification; no audit-only
clock remains in production code.

## Architecture comparison

The upper-rank reference in `.steering/ano` runs the main GMPCC solve directly
at roughly 22--57 ms per update and isolates only tactical alternatives in a
child process.  It does not expose a separately solved normal trajectory to a
later cursor-zero adoption step.  Our normal producer is asynchronous too,
so a different certified predecessor can move the actuator while the next
normal QP is in flight.

The next slice must compare a same-cycle current-world solve of the unchanged
seven-state formulation against the failed asynchronous join.  It is an
observation-only architecture arm first.  Promotion is allowed only if the
same QP, wall proof, dynamic proof and publication proof all succeed from the
actual previous serialized command.  It must replace, not permanently join,
the defective normal-production lifecycle if adopted.
