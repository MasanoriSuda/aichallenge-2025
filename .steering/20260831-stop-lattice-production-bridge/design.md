# Design

## Authority resolution

```text
ordinary current-world retained evaluation
  accepted -> select ordinary normal authority
  rejected -> evaluate current published-source lattice plan once
                accepted -> select lattice normal authority
                rejected -> existing published Stop successor join
                              accepted -> select Stop successor normal authority
                              rejected -> external emergency Stop
```

The lattice plan is not inserted into the candidate Store.  Its immutable
`CertifiedPlan` and current-world `ProductionAuthority` flow through the same
`RateResolvedRetainedShadowEvaluation` value already consumed by the sole
canonical normal adapter.  Publication therefore establishes the next source
lifecycle through the existing publisher-boundary hook.

## Single evaluation

The former shadow observer is replaced by an evaluator returning one typed
result.  The caller either moves that result into the selected `retained`
value or records its reject reason.  It may not re-evaluate the plan for
logging or selection.

## Lifecycle

- Only a mailbox result whose source identity equals the currently published
  Overtake artifact may become the alternate.
- Publishing the alternate as ShiftOut/Pass establishes a new published
  source and naturally schedules its successor lattice observation.
- Publishing non-Overtake or external Stop invalidates the old source and
  alternate.

## Telemetry

Report missing, attempted, selected and typed rejection counts.  The decisive
line must identify ordinary reject reason, lattice source, join reason and
whether it became canonical normal authority.

## Non-goals

- Repairing the later actual-footprint wall-margin violation.
- Changing the Overtake Mission geometry.
- Tuning runtime or solver settings.
- Keeping a stale alternate after the published source lifecycle ends.
