# Rejoin handoff against the current immutable dispatcher

2026-09-13. Authorized autonomous M1–M6 scope; no push. Rollback baseline a865ed4e.

## Earliest invariant and producer

single-r16/D1 decision952 accepts Rejoin source528/job951 with current geometry
7297210173200458920 and current domain proof, then Recovery overrides it with Stop.
The first span has accepted candidates; later source wall failures cannot explain
this first lost handoff. R694 uses the actual ScheduledDispatchFixture, prepare_dispatch and canonical_command, with and without a supplied domain.
Both select the original enclosing tube and fail the legacy
same-decision handoff. DispatchCandidate correctly retains the worker certificate
ID separately from the current dispatch ID; the Recovery predicate incorrectly
requires equality. Overridden final telemetry clears plan/certificate fields, so
those zeros are not the pre-arbitration command. R695 records that detection limit.

## Architecture and changed boundary

R695 compares the same certified candidate through three handoff boundaries.
A: require identical IDs fails both original-tube cases. B: rewriting the old certificate
ID fails the authenticated publication receipt. C: the current opaque dispatcher
checks exact canonical command equality, the live ledger prefix, context generation,
current decision, existing original window and wire/slew guards, then the unchanged
Rejoin semantic/speed conditions. Only C is a candidate for adoption. This is an
identity-interface repair; Mission/solver/formulation/world/model remain unchanged.
The prior source solver architecture comparisons do not provide publication authority.

Remove the standalone scalar identity heuristic and its fabricated positive fixture.
Move the handoff onto DispatchCandidate; share exact field comparison with the final
publication identity. All original source IDs, actuation, fingerprints and packet
values remain unchanged. No new ID, current certificate fabrication, grace, retry,
timeout, speed/margin/physical relaxation or second normal authority is introduced.
The actual publisher still checks immediately before and after send. Recovery Stop,
gear/reverse, first Rejoin hold, actuation enable and supervisor guards are retained.

## Acceptance

Actual certified Rejoin passes original tube, independent physical and domain proof routes with distinct original/current IDs.
Changed command fields, missing/non-Rejoin identity, bad caps, changed current decision,
expired original window, stale ledger and invalidated context are rejected. The exact
actual send receipt preserves both IDs and rejects rewriting. Run focused/native and
package tests, source/architecture checks, make autoware-build, review and local
commit, then single-r17. Success requires actual certified Rejoin publication; it
is not integrated Start/lap/race acceptance. Keep r80 deadline and source/model
failures as separate open work. Record all failed/setup diagnostics and protected
artifact hashes. [Prior supply boundary](recovery-rejoin-supply-design.md).

R696passes the new handoff but exposes a fixture coverage mistake: unchanged current
observations still use the original tube even when a domain is supplied. Keep the
failed output; R697adds a small measured position change and asserts the actual
proof route, while retaining the exact original-tube case. No production change.

R697passes243native tests with three actual proof routes and unchanged negative
expectations; source118pass, build131all26pass with frozen files. Package118:
Summary: 2639 tests, 0 errors, 0 failures, 0 skipped. Final source/current decision receipt test remains unchanged.
No new review finding in changed callers/authority/units or public interface.
Local commit then single-r17for real certified Rejoin; do not infer full recovery
or race acceptance from these local tests. [Validation](recovery-rejoin-handoff-validation.json).
