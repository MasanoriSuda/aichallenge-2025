# Tasklist: terminal retained execution identity

- [x] Freeze and trace the first failing Return lifecycle.
- [x] Identify the self-replenishing authority edge.
- [x] Add explicit live-tactical-state identity to the pure resolver.
- [x] Reject terminal and phase-mismatched retained identities in tests.
- [x] Build and run the complete package tests.
- [x] Run dynamic `make dev2` acceptance.
- [x] Record the audit and commit the Slice.

## Dynamic acceptance note

`output/20260828-011708` contained four overtake episodes.  It did not reach a
successful Return completion, so the exact terminal transition was not
replayed.  It did contain multiple terminal transitions back to Idle, and the
old recursive `Canonical executed-intent replenishment` chain was absent.
The distinct Pass short-horizon and ShiftOut wall failures are frozen for the
next architecture classification rather than being patched in this Slice.
