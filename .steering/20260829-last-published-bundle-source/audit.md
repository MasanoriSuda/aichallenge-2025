# Audit: Bundle-to-next-cycle authority hole

## Causal chain

1. Gate A produces a valid ShiftOut source and current-world proof.
2. Its Bundle command joins the publisher at decision 2629.
3. Correct anti-false-provenance behavior does not mark the source plan
   executed.
4. No separate ledger records the Bundle source and publication cursor.
5. At decision 2630 the normal Store exposes old Cruise sequence 2014.
6. ShiftOut evaluation rejects it as `intent-mismatch` before any physical
   proof.
7. Normal authority is unavailable and Emergency is published at 4.83 m/s.

The observed wall clearance was not the rejecting invariant. Adding a timeout
or retaining Stop would only lengthen the downstream symptom.

## Existing patch relationship

`16a4abec` correctly repairs stage/publisher phase and exposes this next
lifecycle invariant. Its rule that a stateless Bundle must not falsely promote
the unmodified source remains valid. This Slice adds the missing distinct
publication-source ledger rather than reverting that rule.
