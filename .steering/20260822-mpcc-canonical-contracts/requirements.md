# Slice 1: canonical MPCC execution contracts

## Baseline

- Branch: `develop_july`
- Baseline commit: `1ecfe78` (`Document MPCC single-authority migration`)
- Parent audit: `.steering/20260822-mpcc-single-authority-audit/`
- Existing unrelated working-tree change: `aichallenge/result-summary.json`

The existing result summary is out of scope and must not be modified or committed by this steering.

## Repaired invariant

The problem used to produce a normal command, the solution/certificate selected for execution, and
the final published decision must have one explicit immutable identity. An Emergency or Recovery
override must explicitly retain the decision it replaced.

## Earliest current violation

The current pipeline identifies the control cycle with `decision_id`, but formulation and solution
identity are reconstructed downstream from strings such as `extended-mpcc-solved`, target/side
fields, and the selected final-source enum. A retained solution or post-solve hold can replace the
control without a common problem/certificate fingerprint.

This is an observability and contract defect. It does not prove that the current command is unsafe,
but it prevents deterministic attribution of an unsafe or slow command to the exact problem and
certificate that produced it.

## Scope

1. Add immutable `ControlIntent`, `MpccProblemContext`, `CertifiedMpccSolution`, and
   `FinalControlDecision` value types.
2. Add deterministic fingerprints for problem context and solution identity.
3. Distinguish certified normal output, legacy normal bypass, Emergency override, Recovery override,
   and disabled control in the final contract.
4. Preserve source problem/solution identity when a retained Dynamic Escape solution is restored.
5. Join the structured contract to the existing final decision trace.
6. Add focused unit tests for fingerprint stability, mutation sensitivity, certification, mismatch
   rejection, override handling, and legacy-bypass visibility.

## Non-scope

- Do not change solver selection, candidate selection, trajectory, control output, or source
  precedence.
- Do not change runtime configuration or parameter values.
- Do not enable MPCC outside its existing activation conditions.
- Do not remove a normal control path in this slice.
- Do not parse `solver_reason` to infer formulation or certification.
- Do not add a fallback, hold, retry, lease, cooldown, or feature flag.

## Acceptance

- Existing command values and final-source precedence remain unchanged.
- Every final trace contains a structured `FinalControlDecision`.
- A normal solver command can be marked canonical only when context and certified solution
  fingerprints match.
- Every current non-certified normal source is explicitly labeled `legacy-normal-bypass`; it is not
  silently presented as a certified MPCC solution.
- Emergency and Recovery overrides are complete without inventing a solver solution.
- Retained Dynamic Escape execution reports the original solution identity.
- Focused unit tests and package build pass.
- A short `make dev2` trial shows no `identity=incomplete` for published commands; any occurrence is
  treated as a Slice 1 defect, not tuned around.
