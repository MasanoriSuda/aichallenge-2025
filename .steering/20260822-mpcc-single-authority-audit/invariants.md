# MPCC single-authority invariants

## Invariant table

| ID | Machine-checkable invariant | Producer | Checker / telemetry | Baseline status | Known gap or violation | Required regression test |
|---|---|---|---|---|---|---|
| I-01 | A selected normal-driving candidate is `Solved && Finite && ConstraintValid && PhysicallyCertified` | canonical solver/certifier | selection assertion and final trace | Partial | Tactical/admission and executed physical validation are separate stages | Feed solved, unsolved, non-finite and wall-invalid candidates; only the fully certified one may publish |
| I-02 | Selected solution, executed trajectory, wall/opponent certificate and final command share one immutable problem fingerprint | problem builder/certifier | final decision fingerprint equality | Not implemented end-to-end | Mission, DP prefix, converted solution, current prediction and retained execution have separate identities | Replay an async refresh and wall handoff; mismatched result must be rejected before authority changes |
| I-03 | Lateral and longitudinal normal commands derive from the same certified prediction | canonical MPCC | final trace carries one solution ID for both axes | Observed conflict | Runtime reports `multiple-lateral-authorities`; direct/floor/hold owners can differ | Construct a DynamicWait/Recovery boundary and assert one normal solution ID or explicit emergency override |
| I-04 | Track, Cruise, Follow, Hold, Stop, ShiftOut, Pass, Return and Rejoin use one canonical normal formulation | intent/problem builder | formulation enum in every cycle | Violated by design | MPCC is overtake-only; low-speed direct and legacy paths remain | Scenario matrix must show canonical formulation for every normal intent |
| I-05 | A normal solve failure never transfers authority to a different formulation | solver supervisor | formulation transition counter | Violated by design | extended -> 3-state -> legacy fallback exists | Force extended build/solve failure; result must be same-formulation last-certified or emergency stop |
| I-06 | Intent/formulation/horizon/geometry changes invalidate incompatible warm starts | warm-start store | reset reason and context key | Partial | Several reset/handoff mechanisms exist, but no single problem fingerprint governs all | Change each context field independently and assert reset; unchanged context must warm start |
| I-07 | Dynamics, corridor, prediction and physical validation use the same stage geometry convention | `StageGeometry` | geometry hash and stage-by-stage assertions | Partial/improving | Unified foundation exists; legacy consumers and converted output remain | Circular seam, zero segment and heading-gradient replays must produce identical stage coordinates |
| I-08 | Dynamic obstacle prediction is time-aligned, fresh and bound to the selected target/observation generation | obstacle tube/provenance | target provenance validation | Partial | Current target provenance is strong, but all non-target obstacles are not part of one canonical context | Advance observation generation while async solve is running; stale result must not publish |
| I-09 | Only emergency safety may override a moving normal command without another solved trajectory | emergency supervisor | explicit override reason | Not satisfied | wall holds, crawl and direct control are also normal-output sources | Enumerate final source; only canonical MPCC, emergency, or Recovery is accepted after migration |
| I-10 | Recovery cannot silently hide its upstream entry cause | recovery supervisor | recovery entry includes preceding solution/failure fingerprint | Partial | Recovery is separately traced, but normal failure identity is not one canonical fingerprint | Trigger wall/solver/contact entries and assert upstream cause linkage |
| I-11 | Async results are adopted only when the entire problem context matches, not by age/target alone | async worker/result mailbox | context fingerprint comparison | Partial | epoch/target/side/horizon/provenance checks exist, but schema/weights/all bounds are not one key | Modify one context component during solve and assert rejection |
| I-12 | Branch objective values are compared only for the same horizon, state/input schema, weights and terminal semantics | branch evaluator | score schema ID equality | Partial | Dual extended branches are comparable; Hold/Return schema is documented as not fully solver-connected | Mixed-schema candidates must be marked non-comparable, never ranked |
| I-13 | Last-feasible continuation is same-formulation, same-context, physically certified and bounded by time/stages | certified solution store | remaining horizon and expiry in final trace | Partial | Dynamic Escape lease improved identity checks; general normal-control policy is fragmented | Expire time, stage count, target, side and geometry independently; each invalidation must stop continuation |
| I-14 | Every published command has one decision ID, intent, formulation, solution/certificate ID and override reason | final decision publisher | one structured trace per state change/episode | Partial | Decision trace is rich but not backed by a single final solution certificate | For every publisher call, require a complete trace record with no `unknown` provenance |
| I-15 | Schema-only, shadow-only, or unattempted candidates are never executable or reported as selected | branch selection | candidate state enum | Unknown/current spec gap | Race MPCC spec says Hold/Return were not solver-connected at foundation stage | Inject schema-only Hold/Return and assert `NotEvaluated`, not selected |

## Earliest known structural violations

### E-1: Intent-to-formulation split

`progress_contouring_mpcc_overtake_only: true` makes behavior/phase decide which dynamics and cost
schema run. This violates I-04 before any OSQP failure occurs. Solver fallback and command handoff are
downstream consequences.

### E-2: Cross-formulation fallback

An unavailable extended solution is converted into a three-state/legacy solve in the same control
cycle. This violates I-05. Circuit breakers, reentry counters, and handoff smoothing reduce symptoms
but preserve the violation.

### E-3: Split solution identity

Candidate, DP path, extended primal, converted output, current prediction, wall certificate, and
retained execution do not share one immutable identity. This is the upstream design gap behind many
wall-handoff and stale-result patches. Current identity checks repair subsets but not I-02 globally.

## Root causes versus masks

| Classification | Current examples |
|---|---|
| Root architectural causes | intent-dependent formulation; split solution identity; multiple normal command authorities |
| Contributing causes | async delay; narrow corridor; solver conditioning; V2X jitter; course seam; large monolithic state surface |
| Masks | circuit breaker; reentry gate; legacy fallback; wall hold; solver crawl; retained steering; Recovery |
| Detection gaps | no final immutable certificate/fingerprint; no deterministic replay for each transition; incomplete objective schema identity |
| Recovery behavior | emergency stop, Stuck Recovery, reverse/gear recovery; retained only for actual post-failure safety |

OSQP maximum iterations, wall rejection, and Recovery entry are not automatically root causes. They
become root causes only when the input problem and preceding invariants are proven valid.
