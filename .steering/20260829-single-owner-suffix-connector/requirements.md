# Requirements

## Objective

Promote the proven common-clock suffix capability through one bounded
production connector, without adding a second controller, Store or command
publisher.

## Baseline

- Commit: `7eecb011`.
- `output/20260829-182105` proves that a time-aligned suffix can turn an old
  `steering-unreachable` preparation into a fully current-world-certifiable
  seven-state artifact.
- Unrestricted observation submission is rejected as production architecture:
  it continuously occupied a worker and reached 108.11 ms tail cost.

## Admission contract

The connector may claim a source only when all conditions hold:

- normal production authority is currently unavailable;
- the raw certified candidate failed specifically as `steering-unreachable`;
- candidate sequence and preparation sequence are identical;
- the source has not already been claimed;
- no connector is in flight.

The claim is one-shot.  A connector-produced candidate is not recursively
connected.  Other rejection reasons remain fail-closed and do not enter this
path.

## Causal contract

- The source preparation comes from the raw async result.
- Latest state and previous input are bound only after the current callback's
  exact command has crossed the serialization boundary.
- The suffix output receives a new sequence from the same canonical sequence
  allocator as normal jobs.
- The connector uses the existing primary latest-only worker; while it is in
  flight, normal submissions are suppressed rather than run concurrently.
- QP, nonlinear physical, wall, dynamic and current-world retained proof are
  unchanged.
- Only the existing `CertifiedPlan::Store` and normal publisher may grant
  authority.

## Prohibited changes

- no Mission resume rule, retry lease, grace period or timeout;
- no second authority Store or fallback controller;
- no solver tolerance, wall clearance, steering-rate or behavior tuning;
- no recursive connector attempt;
- no adoption based only on solve success or artifact age.

## Dynamic acceptance

- connector submissions are bounded and do not continuously occupy the worker;
- at least one previously unavailable normal authority gap becomes a certified
  normal command through the connector;
- connector candidates still pass unchanged current-world and terminal Stop
  proof;
- no stale source, identity mismatch or recursive claim is accepted;
- no legacy normal command source or second publisher appears.
