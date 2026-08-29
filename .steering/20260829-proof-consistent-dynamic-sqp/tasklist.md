# Task list: proof-consistent dynamic SQP audit

- [x] Freeze production authority and configuration.
- [x] Add observation-only SolverContext entry point.
- [x] Rebuild dynamics, dynamic supports, and wall rows from one primal.
- [x] Add a focused comparison CLI mode.
- [x] Add unit tests proving audit-only separation and row refresh.
- [x] Replay representative dynamic and post-linearization failures.
- [x] Compare with accepted regression snapshots.
- [x] Decide production promotion or reject the hypothesis.
- [x] Build, document, and commit the Slice.

Decision: do not promote a fixed-count outer SQP.  It rescues one frozen
candidate but regresses another already-certified candidate.  The next Slice
must compare each iterate through the unchanged exact proof/merit chain and
retain the last certified artifact; it must not blindly replace it with the
last numerical iterate.
