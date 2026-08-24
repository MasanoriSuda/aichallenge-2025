# Tasklist

- [x] Freeze baseline and preserve the user-owned result artifact.
- [x] Define phase-specific authority and evidence acceptance.
- [x] Verify the installed binary was rebuilt from the frozen baseline source.
- [x] Run source deletion gates and focused authority tests.
- [x] Run `make autoware-build` before the dynamic Gate.
- [x] Execute a bounded `make dev2` dynamic Gate.
- [x] Classify ShiftOut coverage.
- [x] Classify Pass coverage.
- [x] Classify Return coverage.
- [x] Classify DynamicWait/DynamicEscape coverage.
- [x] Audit authority/fingerprint joins and stale adoption.
- [x] Audit physical wall/corridor evidence and Recovery boundary.
- [x] Audit worker/callback timing.
- [x] Mark every unexercised phase `NOT EXERCISED`.
- [x] Reject the Gate at the first source-level invariant violation.
- [x] Update the migration ledger.
- [x] Commit only accepted steering/source/test changes.
