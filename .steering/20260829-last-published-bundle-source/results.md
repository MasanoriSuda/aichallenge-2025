# Results: last published Bundle source

## Root-cause conclusion

This failure family is a publication-lifecycle defect. A stateless
current-world Bundle correctly did not claim that its modified source plan was
executed, but the publisher also discarded the immutable source identity whose
proved command actually crossed the wire. On the next callback, selection fell
back to an older exact-execution source, failed intent identity before the
physical proofs, and inserted Emergency.

The repair records a separate last-published Bundle-source ledger only after
the serialized command matches the final command. It stores source identity and
publication mapping, not an executable lease. Candidate, Bundle source and
older exact-execution source are tried in that order; every attempt must pass
the complete current-world proof. A later exact publication supersedes the
Bundle ledger.

## Static verification

- `python3 -m pytest .../test_single_authority_source_contract.py`: 75 passed.
- `make autoware-build`: 25 packages passed.
- package CTest in the development container: 54/54 targets passed.
- package test summary: 2159 tests, 0 errors, 0 failures, 0 skipped.
- Store regressions cover non-promotion, exact-publication supersession and
  chronology rollback rejection.

No solver setting, tolerance, wall or vehicle clearance, timeout, lease,
retry, fallback or configuration was changed.

## Dynamic A/B

Baseline was commit `16a4abec`, run `output/20260829-223720`. Candidate was
`output/20260829-230250`. The intermediate run
`output/20260829-230020` never left AWSIM `spawned` and is excluded.

| D1 observation | Baseline | Candidate |
|---|---:|---:|
| Gate A ShiftOut admissions inspected | 3 | 3 |
| immediate next-cycle ShiftOut Emergency | 3 | 0 |
| periodic accepted `published-bundle` source windows | 0 | at least 3 |
| Bundle source record rejection | N/A | 0 |

Baseline decisions 2023, 2629 and 6025 all published a ShiftOut Bundle and
immediately fell to Emergency at 2024, 2630 and 6026. Candidate decisions 1439,
2947 and 3091 all retained normal authority on the next cycle. Periodic
telemetry independently observed accepted `published-bundle` revalidation for
source sequences 964, 2736 and 2894.

This closes the frozen 2629 -> 2630 class without relaxing a proof. It also
supports the A/B/C/D classification `A fails, B succeeds`: persistent exact
execution provenance alone loses the publication source, while stateless
current-world rebuilding from the last published Bundle source succeeds.

## Separate remaining failure family

Candidate decisions around 2984, 2988, 2992 and 2995 still entered ShiftOut
Emergency after roughly 1.1--1.4 seconds of execution. Exact candidate
authority reappeared between those events, and subsequent Stop diagnostics
reported `terminal-contingency-unavailable`, `continuation-wall-blocked` and
`delay-prefix-blocked`. This is not the immediate Bundle-source loss repaired
here. It remains a distinct terminal/candidate proof family for the next
root-cause Slice; adding another lifecycle patch here would conflate causes.
