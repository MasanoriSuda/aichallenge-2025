# Design: include published Stop in atomic admission

## Root cause

The controller maintains `last_published_canonical_intent_`, intentionally a
ledger of normal MPCC owners. SafetyBrake does not overwrite it because Stop
shadow planning still needs to know which normal intent was interrupted.

The same ledger was also used as the previous owner for atomic intent
admission. After Stop had actually owned the wire, admission therefore saw an
obsolete Cruise owner instead of Stop. When the proposed Follow artifact was
one worker cycle late, neither proposed Follow nor obsolete Cruise passed
current-world proof and the controller created a generic Emergency.

## Repair

1. Keep the normal-intent ledger for shadow-successor selection.
2. Add a separate published-authority ledger that records normal intents and
   explicit Stop.
3. Extend atomic admission so a proven previous Stop can remain authoritative
   when a proposed normal intent is not yet ready.
4. If atomic admission retains Stop, publish the same explicit Stop authority
   and continue submitting the proposed normal successor.
5. Replace Stop with the normal intent only after that exact normal command is
   serialized successfully.
6. Record the semantic intent passed to an Emergency result, rather than
   reconstructing it from the current FSM label.

## Why this is not a fallback

Stop is already the published authority. The change neither invents a new
trajectory nor revives an old normal command; it prevents an authority change
until its replacement exists. This is the same atomic handoff invariant used
between normal intents, extended across the external Stop boundary.

## Alternatives rejected

- Relax the Follow initial hard gap so the shadow solves during SafetyBrake:
  violates the physical separation contract.
- Wait a configured number of cycles after SafetyBrake: a timing grace cannot
  prove successor authority.
- Reuse the old Cruise command: its steering predecessor was no longer
  reachable after the Stop command and current-world revalidation correctly
  rejected it.
- Accept one generic Emergency cycle: leaves the authority ledger inconsistent
  and repeats for any slower worker or longer Stop interval.

## Deletion boundary

Atomic transition admission no longer treats the last normal intent as a proxy
for the last published wire owner. The two responsibilities remain explicit.
