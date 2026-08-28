# Tasklist

- [x] Freeze hypothesis and production invariants.
- [x] Add explicit observation-only bucket policies.
- [x] Add comparison arms and CLI mode with no production path.
- [x] Add deterministic unit/source-contract coverage.
- [x] Run focused tests and build.
- [x] Replay frozen fingerprint `a6f7c37f1de517c1`.
- [x] Reproduce the classification on independent live fingerprint
  `145d1159f38a6ea9`.
- [x] Restore the original racing objective after Phase-I feasibility and
  distinguish physical feasibility from objective convergence.
- [x] Record the classification without changing production authority.

The audit entry points remain observation-only until the immediately following
numerical-formulation/backend Slice consumes this evidence.  That Slice must
either promote one proven formulation while deleting the replaced hard-bucket
path, or delete these audit entry points if no formulation is promoted.
