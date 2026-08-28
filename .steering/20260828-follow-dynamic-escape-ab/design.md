# Design

The existing stateless candidate builder deliberately accepts only
ShiftOut/Pass/Return because it is also used by the production Overtake worker.
Relaxing that production contract would silently expand authority.

Instead, factor the pure builder behind an internal intent policy and expose a
separate `build_follow_escape_audit` entry point. It accepts only Follow,
rebuilds the target tube from the sealed ReplayWorld, proposes one explicit
side, and returns data for the unchanged solver/proof comparison. The existing
`build` function retains its Overtake-only rejection contract.

For a Follow snapshot, architecture comparison emits only:

1. persistent Follow A;
2. stateless positive-side B;
3. stateless negative-side B.

This avoids thousands of meaningless Overtake-lattice rejection rows and
answers the first causal question before adding C or D.
