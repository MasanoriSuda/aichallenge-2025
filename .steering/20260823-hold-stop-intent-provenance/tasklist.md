# Task list

- [x] Audit current DynamicWait/SafetyBrake owners and representative logs.
- [x] Add failure-first canonical-intent tests.
- [x] Implement the pure intent resolver and typed reasons.
- [x] Add DynamicWait origin phase to authority provenance.
- [x] Replace the controller-private action switch.
- [x] Build and run focused tests.
- [x] Verify authority trace output dynamically.
- [x] Audit no command/config/fallback changes.
- [x] Commit without staging user-owned result data.

## Definition of Done

- Rolling DynamicWait retains ShiftOut/Pass intent.
- Lateral-only and rolling DynamicWait preserve committed ShiftOut/Pass intent.
- SafetyBrake maps to Stop.
- Invalid semantic combinations map to Unknown with a typed reason.
- Production behavior and command authority remain unchanged.

## Dynamic evidence disposition

`output/20260823-132619` verified that the final published-command trace now
joins `canonical_intent` and its typed reason with the execution contract for
Track, Cruise and Follow. The short run did not enter DynamicWait. Rolling
and lateral-hold DynamicWait therefore remain covered by historical evidence
and deterministic resolver tests; this Slice does not promote Hold/Stop or
claim positive runtime coverage with the new trace.
