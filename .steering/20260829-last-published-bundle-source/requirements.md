# Requirements: last published Bundle source

## Objective

Remove the normal-authority hole immediately following a successfully
published stateless current-world Bundle. Preserve only its immutable source
identity and publication clock, then rebuild and prove the next command from
the current world. Do not retain a Mission path or grant authority by age.

## Frozen evidence

- Baseline commit: `16a4abec`.
- Dynamic run: `output/20260829-223720`.
- Decision 2629 published a certified ShiftOut current-world Bundle from Gate
  A plan sequence 6.
- Decision 2630 retained only stale Cruise plan sequence 2014. Revalidation
  returned `intent-mismatch`; no normal source remained and Emergency was
  published at 4.83 m/s.
- The same bounded run contains five initiating ShiftOut authority holes and
  six Cruise authority holes. The Cruise terminal-contingency failures remain
  a separate family and are not part of this Slice.

## Constraints

- No Mission resume rule, lease, grace, timeout, retry or fallback.
- No solver, clearance, wall, dynamic-obstacle or terminal-proof change.
- Do not mark a modified Bundle command as an exact execution of its source
  plan.
- Persist only target/homotopy provenance, source plan identity, publication
  clock and source cursor of the last command that actually joined the wire.
- Every later command requires a fresh current-world proof.
- A failed proof remains Emergency; publication history is not authority.

## Definition of done

- A serialized Bundle atomically records its immutable source and source
  cursor as last-published evidence.
- The next cycle can evaluate that source before an older executed plan.
- A newly published exact plan supersedes the Bundle source.
- Failed or non-joined commands never update the ledger.
- Unit/source-contract/full tests and bounded dynamic A/B pass.
- The 2629 -> 2630 class no longer inserts a ShiftOut Emergency without
  relaxing any proof.
