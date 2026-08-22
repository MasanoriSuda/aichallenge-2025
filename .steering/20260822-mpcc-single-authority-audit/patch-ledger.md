# Patch ledger

This ledger groups the major control-flow patch families. It is intentionally organized by authority
and invariant rather than listing every conditional in the 49k-line controller. `Unknown` means the
history is too opaque to infer intent safely and must be resolved before deletion.

| Family / representative symbol | Introduction or key commits | Original symptom/intent | What it currently masks or amplifies | Target invariant | Current disposition | Deletion gate |
|---|---|---|---|---|---|---|
| Progress MPCC activation / `progress_contouring_mpcc_overtake_only` | `de96953` | Add physical progress during overtake without replacing normal MPC | Makes formulation depend on phase; preparation failure returns to legacy per cycle | I-04, I-05 | Temporary migration path | Canonical MPCC passes Track/Cruise/Follow and overtake replay |
| Five-state extended MPCC | `0863306`, `6f27c87` | Couple acceleration, curvature, speed and virtual progress | Its output is converted to the older layout and can lose authority during requalification | I-02, I-04 | Keep as canonical candidate | Direct certified output reaches publisher without legacy conversion |
| Extended circuit breaker | `6f27c87`, later hardened by `5b80c03` | Prevent repeated expensive failed solves | Delays visibility of repeated infeasible problem construction and hands control to an older formulation | I-05, I-13 | Temporary | Same-formulation last-certified policy and deterministic failure classification exist |
| Extended reentry gate | `843142b` | Avoid one lucky solve chattering with 3-state MPCC | Preserves two normal authorities and introduces a mode-dependent acceptance window | I-05, I-06 | Temporary | No cycle-local formulation fallback remains |
| Extended mode handoff smoothing | `1230b74` | Smooth command discontinuity between formulations | Treats a structural mode switch as a signal-smoothing problem | I-03, I-05 | Temporary | One normal formulation owns consecutive cycles |
| Dual left/right extended branch | `013ef38` | Compare both pass homotopies with a common solver | Result remains tactical until separate admission/execution paths accept it | I-02, I-11, I-12 | Keep capability, simplify ownership | Both branches carry immutable context and selected result becomes one certified solution |
| Async latest-only tactical worker | `593fcc7`, `8a71857` | Keep heavy branch evaluation out of 40 Hz callback | Cached and current contexts can diverge without complete fingerprints | I-11 | Keep worker | Context fingerprint covers observation, geometry, target, intent, bounds schema and weights |
| Last-feasible MPCC execution | `e8c4102`, `f249112` | Continue through a transient optimizer miss | Can become a lease that hides stale topology/context if identity is partial | I-02, I-13 | Keep concept, narrow contract | Only same-formulation certified solution with bounded horizon and exact context compatibility remains |
| DP execution handoff / rolling prefix | `b335f57`, `fb2e508`, `5b80c03` | Execute receding corridor instead of frozen Mission path | Planner prefix and solved trajectory can each appear to own execution | I-02, I-03 | Convert to MPCC input | DP supplies constraints/reference only; MPCC certificate owns command |
| Executed physical wall validation | `9618f40`, `0561f57` | Catch solver trajectory that differs from entry-certified path | Late rejection can hide that planning and solving used inconsistent geometry/provenance | I-02, I-07 | Keep safety check, move into certificate | Solver output is certified with the same geometry and fingerprint before selection |
| Wall handoff/admission gates | `7fe740c`, `d78cba9`, `0561f57` | Avoid publishing a wall-unsafe incoming path during handoff | Requalification can insert braking/hold between otherwise valid solves | I-02, I-13 | Temporary stateful bridge | Atomic certified replacement or bounded same-solution continuation needs no separate handoff gate |
| Low-speed direct control / `low_speed_shift_control_active_` | first visible in `a3929d7`; early history has WIP commits | Escape low-speed solver/rejoin failures | Bypasses canonical solver and creates another steering/velocity owner | I-03, I-04 | Remove in later slice | Follow/Hold/low-speed obstacle intents pass canonical MPCC replay |
| Solver fallback speed/crawl | early history includes `bfe8aaf`, `a3929d7`; later wall validation in `9618f40` | Avoid indefinite stop or unsafe raw solver output | Changes final symptom from solve failure to crawl/stop and may trigger Recovery later | I-05, I-13 | Replace | Same-formulation last-certified continuation plus emergency stop is verified |
| Solver reentry/cooldown | early commits are `wip`; current logic around `mpc_controller_cpp.cpp:21146` | Avoid immediate reentry after repeated failure | Failure is expressed as time/cycle policy rather than invalid-problem classification | I-05, I-06 | Audit before removal | Problem class is deterministic and incompatible warm starts are invalidated |
| SafeSeparation/forward completion | `10b3420`, `7304601` and later pass patches | Prevent early Recovery/Return while pass can finish | Many completion exceptions can keep an invalid Mission alive or obscure missing terminal intent | I-04, I-12 | Re-express as intent/cost | Canonical Pass terminal/rear-clear constraint covers the scenarios |
| Dynamic Escape candidate backoff | `b736b99` | Avoid repeating a failed side/candidate | Can quarantine a valid side when failure was caused by outgoing prediction provenance | I-02, I-11 | Temporary | Failure attribution uses the certified solution fingerprint and candidate identity |
| Dynamic Escape lifecycle | `ede406a`, `42da7ed` | Keep one encounter alive across planner-request gaps | Correctly improves continuity, but remains separate from solution validity | I-08, I-13 | Keep encounter concept | Lifecycle is intent state only and cannot extend solution/certificate validity |
| Dynamic Escape retained execution lease | `d78cba9`, `b079830`, `dc51093` | Prevent fresh-candidate gaps from dropping lateral authority | Necessary while candidate and execution lifetimes are separate; still adds handoff states | I-02, I-13 | Keep until canonical solve owns every cycle | Receding canonical MPCC or bounded same-certificate continuation replaces the lease |
| Central decision trace/authority resolver | `db430a1`, `4806298`, `6bc809f` | Make final owner and transition reasons observable | Does not itself enforce one solver/certificate authority | I-14 | Keep and generalize | Trace is emitted from `FinalControlDecision` and uses canonical fingerprint |
| Stuck Recovery | multiple July recovery commits; current spec marks it separate | Recover from stopped/contact/gear conditions outside forward model | Can mask normal-control wall departure if analysis begins at Recovery entry | I-09, I-10 | Keep separate | Recovery entry records the upstream normal-control failure fingerprint |

## Patch concentration finding

The recent history contains a long sequence of authority, handoff, wall, retained-prefix, and Dynamic
Escape fixes. Individually these patches often protect a real invariant. Collectively they form a
network because candidate, solver, certificate, and command are not yet one immutable object.

The migration must therefore delete or demote patch families as each invariant moves upstream. A
slice that only adds another guard or feature flag does not reduce this ledger and is not convergence.

## History limitations

Some early symbols were introduced by commits named `wip`, `仮コミット`, or broad Gate2 changes.
Their original intent cannot be asserted from commit titles. Before deleting those branches, inspect
the full diff/blame and reproduce the failure they addressed. They are marked temporary/unknown, not
automatically obsolete.
