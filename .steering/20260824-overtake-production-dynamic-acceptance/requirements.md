# Slice 5 production dynamic acceptance requirements

## Evidence boundary

- Branch: `develop_july`
- Baseline commit: `174b2682312765743f92563f1bf25f039a80ab3f`
- Existing unrelated working-tree change: `aichallenge/result-summary.json`
- Evidence class: AWSIM simulation (`make dev2`), not real-vehicle validation

The generated result summary is user-owned and must not be edited or committed by this Slice.

## Objective

Close the remaining dynamic-evidence gap after canonical Overtake production promotion. Exercise
`ShiftOut`, `Pass`, `Return`, committed `DynamicWait`, and validated `DynamicEscape`, and prove that
each normal command is produced by one current-world-certified five-state MPCC solution.

## Expected behavior

```text
fresh certified canonical Overtake
-> retained canonical Overtake re-certified against the current world
-> explicit canonical Emergency
```

No Overtake execution state may transfer to converted five-state output, three-state progress MPC,
legacy MPC, low-speed direct control, or a legacy wall-hold owner.

## Scope

- Record intent, target, Mission generation, problem/solution/plan/certificate identity and final
  authority for every exercised Overtake phase.
- Measure async worker result age and compute time, callback overruns, wall/corridor rejection,
  Emergency duration, phase transitions and Recovery entry reason.
- Distinguish a missing dynamic observation from an implementation defect.
- If a Gate fails, identify the earliest violated invariant before changing production source.

## Non-scope

- No parameter, wall margin, clearance, solver tolerance, weight or cadence tuning.
- No new timeout, lease, retry, fallback, hold, suppression or feature flag.
- No attempt to improve lap time or pass aggressiveness.
- No physical deletion of repository-wide legacy code; that belongs to Slice 6 after this Gate.
- No redesign of Stuck/gear/reverse Recovery.

## Acceptance

- `ShiftOut`, `Pass`, and `Return` each have positive runtime coverage.
- Exercised committed `DynamicWait` and `DynamicEscape` retain their Overtake intent/provenance or
  fail explicitly; they do not borrow another normal formulation.
- Every published normal Overtake command has a complete five-state problem, solution, immutable
  plan, current execution certificate and matching final decision identity.
- `legacy-normal-bypass`, converted/three-state ownership, low-speed direct ownership and legacy
  wall admission are zero within the accepted Overtake scope.
- Stale or unsafe evidence is rejected before selection.
- No callback duplicate solve exists; the latest-only async worker remains the sole Overtake solve
  producer.
- Static deletion gates, package tests and build pass.

Absence of a phase in the run is `NOT EXERCISED`, not a pass.
