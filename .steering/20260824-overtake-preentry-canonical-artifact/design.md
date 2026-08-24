# Design

## Root cause

The pre-entry tactical worker already solves independent left/right five-state MPCC problems and
physically certifies the exact selected trajectory. Its API then reduces the winner to scalar
metrics and an Overtake Mission. The canonical command/plan derived from the same primal is not
crossed through that boundary.

The live FSM commits `Idle -> ShiftOut`, and only then starts the separate Overtake canonical
worker. Producer latency therefore creates an unavoidable first `async-pending` Emergency cycle.
Intermittent current-world rejects then brake the vehicle, progress becomes discontinuous, later
worker solves hit maximum iterations and Stuck Recovery masks the original boundary defect.

## Causal classification

- Root: lossy producer/consumer boundary between the pre-entry dual five-state solve and canonical
  production authority.
- Contributor: current-world corridor/progress rejection after braking.
- Mask: explicit Emergency followed by Stuck/AWSIM Recovery.
- Detection gap: branch telemetry proved physical trajectory feasibility but did not report whether
  an executable canonical artifact survived entry.
- Recovery: separate safety action, not the root fix.

## Selected design

1. Adapt each already-solved dual branch to a canonical plan in its isolated worker snapshot.
2. Carry only the immutable plan of the selected branch with the selected Mission.
3. At the live atomic entry boundary, validate plan completeness, cursor lifetime, intent,
   prospective Mission generation, target and side.
4. Freeze the Mission, prepare the matching Overtake canonical lifecycle, and store the plan before
   changing the phase.
5. Keep the existing canonical worker as a rolling refresh producer after entry.

The executable artifact also seals the exact lateral lower/upper state bounds used by the selected
solve. Retained proof slices those bounds at the plan cursor and intersects them with the current
wall grid and current target tube. It must not substitute a later Mission's regenerated trust
corridor for the corridor under which the immutable plan was solved.

Current-world proof remains mandatory on the first production cycle. Pre-entry adoption is not
publication and cannot bypass wall/target/corridor revalidation.

## Rejected alternatives

### Add a grace/hold after ShiftOut

Rejected. It masks missing authority and preserves the duplicate producer boundary.

### Solve synchronously at entry

Rejected. It duplicates work and moves unbounded solve latency into the 40 Hz callback.

### Wait for the existing second worker before entry

Safer than the current behavior, but rejected as the primary design because it retains two
producers solving the same formulation and keeps the original lossy boundary.

### Relax retained proof or wall/corridor checks

Rejected. The runtime failures are downstream evidence, not authority to weaken physical proof.

## Deletion effect

No new normal authority is introduced. The structural change removes the entry-time state in which
a certified dual branch exists but no canonical executable artifact exists. A later Slice may delete
the redundant initial solve role from the production worker; this Slice retains it only as rolling
refresh after entry.

The bounded dynamic Gate separated a later runtime-replan problem: after the initial selected plan
was adopted successfully, the receding-DP source expired and a cross-side Mission replacement again
entered before a matching canonical artifact was available. Progress discontinuity and initial
corridor rejects then alternated with retained commands. That later replacement lifecycle is not
an extension of this entry-artifact fix and is recorded as the next root-cause Slice.
