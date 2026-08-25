# Requirements

## Objective

Identify the earliest invariant that turns a successfully admitted six-state
ShiftOut into execution-source loss, explicit Emergency and later
Recovery/FollowPrepare.

## Evidence baseline

Use `output/20260825-231050`. Gate A itself is accepted and must not be changed:

- three `gate=six-state-shiftout` commits;
- three solved/physically accepted atomic ShiftOut admissions;
- seven certified six-state ShiftOut publications;
- later DP source loss, Emergency and 43 callback overruns.

## Constraints

- no parameter, timeout, lease, wall-margin, horizon, weight or solver tuning;
- no new fallback or restoration of five-state authority;
- do not conflate a stale execution path, absent CertifiedPlan, target loss and
  callback overrun without joining their timestamps and identities;
- modify code only after the earliest causal defect is explained;
- preserve `aichallenge/result-summary.json`.

## Definition of Done

- the data flow from Gate-A proposal to Mission path, six-state solve,
  physical store, retained proof and final command is traced;
- source lifetime and target lifetime owners are identified;
- overrun is classified as causal or independent;
- a failure-first test captures the confirmed structural defect;
- one coherent correction is built, fully tested and dynamically verified;
- remaining path-quality or Pass/Return debt is reported separately.
