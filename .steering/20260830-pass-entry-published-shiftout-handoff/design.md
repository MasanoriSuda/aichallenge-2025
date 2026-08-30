# Design

## Root cause

The tactical phase and the normal command publisher intentionally advance at
different times. Atomic admission keeps a certified ShiftOut command active
until a current-world Pass command is ready. The Pass-entry physical gate,
however, requested the published ShiftOut alignment only while the tactical
phase was literally ShiftOut. It therefore discarded the certificate during
the exact handoff interval atomic admission was designed to protect.

The resulting Stop is downstream:

1. tactical phase becomes Pass;
2. canonical effective intent remains ShiftOut (`previous-retained`);
3. published ShiftOut alignment is not queried;
4. Pass wall warning has no canonical physical certificate;
5. Mission enters DynamicWait/FollowPrepare and loses normal authority.

## Structural correction

Treat tactical ShiftOut and its immediate tactical Pass successor as one
identity-compatible query domain for a published ShiftOut execution artifact.
The existing execution-source adapter remains the proof boundary: it accepts
only the executed ledger entry with exact ShiftOut intent, target, generation,
side, immutable source time, and an available publication cursor.

This is not a time-based retention rule. Once Pass is actually published, the
ledger no longer contains ShiftOut and the query fails by intent. Exhausted or
identity-mismatched ShiftOut artifacts also fail closed.

## Non-goals

- Extending artifact lifetime.
- Weakening wall or opponent proof.
- Changing Pass/ShiftOut geometry or speed.
- Adding another Mission resume/fallback path.

