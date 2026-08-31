# Design: Stop producer/consumer latency

The existing certified plan already contains immutable execution identity,
snapshot time and control prediction origin.  Copy only those values into the
existing telemetry window at the exact point where the current-world alternate
is consumed.  Do not add state to the plan, worker, mailbox or authority
ledger.

The joined record answers:

- how many control decisions separate producer and consumer;
- whether producer and consumer have the same intent and generation;
- how old the source observation is at consumption;
- whether the candidate control origin is already in the past;
- which retained revalidation reason rejected the plan.

If the source is old before consumption, the next hypothesis is worker/mailbox
scheduling.  If it is current but still unreachable, the next hypothesis is
model/certificate join mismatch.  If no accepted plan exists, candidate or
solver failure remains the cause.

The comparison must not equate age with causality.  A plan older than the
failed sample that still passes current-world validation disproves raw age as
the sufficient cause.  Intent/generation mismatches are classified separately
from physical steering reachability.
