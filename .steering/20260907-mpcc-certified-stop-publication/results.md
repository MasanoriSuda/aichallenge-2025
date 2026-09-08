# Certified Stop publication repair (dynamic acceptance pending)

The two-tick execution-ledger test fails before repair: the certified plan is
absent immediately after publishing authority=Stop. The repaired lifecycle
retains the same plan, first control origin and artifact cursor. External Stop
on the next decision still clears the old clock.

The current serialized commit ID is written only after final wire equality and
execution-ledger admission. A Stop bearing that current commit keeps the existing
ledger. A retained executed Stop keeps Stop authority while its immutable source
intent remains provenance. Sibling tactical adoption and generation of further
Stop candidates remain excluded for Stop publication. Current-world proof and
normal-candidate replacement still follow their existing paths.

Verification: 77 execution-contract tests, 31 ledger tests, 103 source contracts;
full package 2312 tests, zero errors/failures/skips. Build and bounded dev2 remain
pending. The preserved pre-repair comparison binary is running A/B/C/D on source
1905 to classify the repeated ShiftOut boundary independently of publication.
No replay of the missing historical live artifact is claimed.

Build completed: 25 packages, 4 min55 s. The architecture comparison found348
certified arms including existing same-side stateless B and production G. This
falsifies a new-generator requirement for the frozen world. Historical async
availability is not reproduced. Summary under
output/20260907-mpcc-stop-publication-architecture/summary.json.

Third dev2 diagnostic started with controller SHA-256
837b58818463605176d29e22dd3f4362a415f8cecae57909bd8d569fa86bde97,
run20260907-mpcc-stop-ledger-dev2. Source patch and both binaries are preserved
in the run directory. Integration outcome pending.

Run3 stopped at2026-09-07 23:59 JST after repeated authority losses. No certified
Stop successor was selected, so this does not test multi-tick Stop publication.
D1 remained ShiftOut for35.75s, then failed terminal wall proof at2677 and
entered Recovery. Eight stateless sibling adoptions were published. A separate
producer defect stops all Pass draft requests immediately after the first
adoption clears frozen Mission state; see ../20260908-mpcc-stateless-successor/.
No formal finish JSON is expected from this manually stopped unlimited dev run.
