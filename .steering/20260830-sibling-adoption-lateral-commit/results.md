# Results

## Static verification

- `make autoware-build`: passed, 25 packages built.
- `test_mpcc_overtake_sibling_adoption`: 13 tests passed.
- `test_single_authority_source_contract.py`: 91 tests passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: passed.
- `colcon test-result --verbose`: 2,138 tests, 0 errors, 0 failures,
  0 skipped.

## Dynamic verification

Run: `output/20260830-191617`, `make dev2`.

- `Idle -> ShiftOut`: 5 observed episodes.
- `ShiftOut -> Pass`: 3 observed episodes.
- `Pass -> Return -> Idle`: 2 complete chains.
- `Published stateless sibling Bundle adopted`: 0 occurrences.
- `committed pass longitudinal progress stalled`: 0 occurrences.
- The first two complete Pass episodes kept side `+1` from ShiftOut through
  Return; neither transient execution-source rejection nor current-world
  revalidation changed the tactical side.

The run did not produce a live opposite sibling exactly while the new
physical-establishment predicate was true, so the explicit
`selected-homotopy-established` rejection reason was not emitted.  That
publication race is covered by the pure resolver and publisher-token tests;
the dynamic evidence establishes that the former post-ShiftOut side mutation
did not recur.

## Independent remaining evidence

Episode 4 reached `ShiftOut -> Recovery` for
`actual footprint wall margin violated`.  Episode 5 remained in ShiftOut at
run shutdown and performed a pre-commit tactical side change.  These occur
before the selected homotopy is physically established and are not caused by
the post-commit sibling-adoption defect fixed in this Slice.

No wall margin, solver tolerance, timeout, lease, fallback or Mission budget
was changed here.  The wall/ShiftOut evidence is frozen for a later root-cause
Slice rather than hidden by this fix.

## Conclusion

The change satisfies the structural objective: solver availability may still
select a sibling before physical commitment, but cannot replace a homotopy
that is already established in the current world.  Static contracts pass and
the previously observed Pass-time cross-track mutation did not recur.
