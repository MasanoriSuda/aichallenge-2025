# Certified Stop publication continuity

User-authorized autonomous MPCC completion. Existing source/test/config changes
and all earlier evidence remain. No solver/tolerance/margin changes in this slice.

A certified Stop which crosses the serializer must remain eligible as the exact
executed plan on the next tick, with its original publication clock and source
identity. Its published authority remains Stop, not ShiftOut/Pass. An external
Emergency/Recovery or mismatched final serialization invalidates that ledger.
Every subsequent command still requires current-world identity, reachability,
wall/dynamic and terminal proof within the original finite artifact horizon.
No lease, grace, direct control or extra retry/authority is introduced.
