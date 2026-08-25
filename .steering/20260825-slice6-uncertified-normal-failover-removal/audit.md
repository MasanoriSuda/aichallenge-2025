# Audit

## Evidence boundary

- Branch: `develop_july`
- Baseline: `f60aba0`
- Evidence: source reachability, git history from preceding migration Slices, deterministic unit/source
  contracts and canonical Docker build
- Preserved user artifact: `aichallenge/result-summary.json`

## Expected versus actual

Expected: every normal racing command comes from one solved, finite, constraint-valid, physically
certified canonical MPCC artifact. Solver/preparation failure may invoke Emergency or Recovery, but
must not select another normal controller.

Actual before this Slice: three helpers still emitted positive-speed normal-like commands without a
complete canonical solution identity:

1. simulation solver crawl generated path-tracking steering and a configured positive speed;
2. Dynamic Escape bounded continuation retained fallback steering and current speed;
3. qualification hold cleared the failed candidate and fallback identity, then copied the previous
   ordinary command for one cycle.

The final contract labelled these outputs `LegacyNormalBypass`, which made the missing canonical
identity observable but still permitted publication.

## Causal classification

- Root cause: an MPCC failure could transfer normal longitudinal/lateral authority to handwritten
  post-solver commands.
- Contributor: simulation-only configuration made the crawl enabled in both checked-in YAML files.
- Mask: `LegacyNormalBypass` represented incomplete normal identity as an accepted final class.
- Detection gap: final classification occurred after post-processing, so the publisher did not
  require typed canonical command evidence before treating a command as normal.
- Recovery behavior: Emergency deceleration and Stuck/gear/reverse Recovery are legitimate
  supervisors and are retained.

## Repair and deletion

- Solver/preparation failure now remains on the existing Emergency deceleration path.
- Missing canonical command identity fails closed as Emergency.
- Solver fallback and executed-solution wall hold classify as `EmergencyOverride`.
- Crawl, bounded continuation, qualification hold, their YAML/config/API/telemetry and
  `LegacyNormalBypass` were physically deleted.
- Final authority classification is a pure tested contract with only certified normal, Emergency,
  Recovery and disabled outcomes.

No new runtime flag, fallback, timeout, lease, margin or parameter was added.

## Remaining unknown

The normal path is unchanged, but this Slice intentionally changes behavior during an actual
solver/preparation failure. Unit tests prove fail-closed authority and build compatibility; the next
dynamic run should confirm that any observed solver failure produces
`authority=emergency-override` and never a positive-speed noncanonical command.
