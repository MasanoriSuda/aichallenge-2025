# Earliest lifecycle break

Run20260907-mpcc-stop-ledger-dev2 D1 enters ShiftOut at decision1251.
Decision1695 /1788793015.080026484 publishes opposite-side Bundle1094,
changes side -1 to +1, and correctly retires mission_plan/mission_path_frozen.
This is the last Pass draft/publication submission. The pending old-side result
448 finishes at1788793018.596113727 and is correctly rejected.
Pass completion later reaches its physical boundary but no new proposal exists.
Eight sibling adoptions occur; ShiftOut stays active35.75s and fails terminal
wall proof at2677, then enters Recovery at2681.

Producer mismatch: draft admission and worker Pass/Return builders still
require frozen Mission state, which the stateless publisher deliberately
removes. Old Mission geometry must not be restored merely to satisfy these
conditions. Use the existing publisher-bound stateless encounter as an
alternative reference source and build prospective geometry from current
world. Retain the Return preflight and all downstream identity/proof gates.
Pass worker must not dereference the absent Mission or copy an obsolete
selected_mission into the stateless snapshot.

Source deletion regression first; A/B/C/D compare sealed2677 before production
repair. Earlier1905 comparison already finds B/G feasible, but is a different
world. All failure without impossibility certificate remains Unknown.
Acceptance: draft generation continues after stateless adoption; Pass/Return
are independently certified and published. Tests/build alone are local checks.
Rollback: revert this slice only against baseline, preserving prior accepted
time/Stop formulation and publication-ledger changes.
