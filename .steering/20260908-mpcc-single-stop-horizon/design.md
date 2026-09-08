# Producer correction and observation boundary

The regression MaximumBrakingStopRejectsUnreachableTerminalRest fails before
repair: a2m/s source with0.3s horizon and-2m/s2 limit is incorrectly Accepted.
The factory overwrites only the final velocity after constructing a reachable
law. Remove that overwrite. If the law still has positive terminal speed,
return InvalidBrakingEnvelope before QP submission. No horizon extension or
changed peer prediction is permitted in this production producer.

Two old tests used that impossible0.3s fixture to test rebasing/determinism.
Use the existing1.2s stoppable fixture and check every velocity/acceleration
transition, including the last. This does not weaken physical acceptance.
The declared-lateral comparison keeps its short fixture: its normal arms still
pass, while both full-Stop arms must now reject construction with no candidates.

Historical observation slice (subsequently retired by stop-reference-feasibility): move the already-existing artifact-local profile copy from
the standalone comparison into the physical adapter, without changing its
validation. A private retained evaluator accepts that profile only through
observe_normal_path_stop; its public observation result contains no Proof,
plan or command. Production evaluate still explicitly supplies nullptr and
keeps the original zero-lateral target. On terminal-proof failure, the
controller logs the paired observation with selected=0 and uses the original
production result. All current-world/identity/delay/actuator/wall/peer checks
remain common. The added measurement is included in total callback duration.

Observation promotion/deletion boundary: collect a bounded single-vehicle
run and compare the exact failing retained plan. If no moving failure is
observed, allow at most two additional identical six-lap runs with distinct
run directories and unchanged binary/config. This bounds investigation of
the intermittent last-lap failure; lack of recurrence does not erase it.
Only propose promotion after
positive live comparison and a full stopping/authority regression; otherwise
delete the observation call when the hypothesis is falsified or the audit ends.
No extra Stop law can execute through this measurement API.

The standalone --target-free-stop-horizon-only comparison admits no target,
tube or peer. It uses the immutable source's existing maximum stage dt, and
reseals a new candidate while retaining original source identity. It changes
no world, bounds, model or proof. Its promotion boundary is NONE: the live
worker does not call this helper. Removing the comparison removes all retiming.

Single11046 results before correction: original1.79s Stop is numerically
inconsistent; offline5s Stop solves but fails exact wall proof. In contrast,
normal-path terminal Stop and bounded lateral-target scan pass the same cold
normal candidate's exact wall/dynamic proof. B/C/D overtake architectures are
inapplicable without a target. The prior executed sequence10415 is missing,
so these accepted cold alternatives do not replay its current-world lifecycle.

Earliest confirmed producer defect: impossible terminal-speed overwrite.
Live wall cause: unresolved stop-reference representation/current-world
execution, with normal-path profile a supported alternative, not yet promoted.
Downstream: solver rejection masks bad construction; live Emergency and stuck
Recovery follow terminal proof failure. They are separate from the unexecuted
optimized Stop factory in this Cruise run.

Validation: focused before/after regression; all architecture tests; full
package/build; frozen short source must reject at construction; long Stop4166
must retain its previously certified outcome. Roll back only this slice to
pre-audit source/binaries (baseline a8b968ac2e4a86a28432a52d132774459d355cb5),
preserving prior accepted repairs. Do not claim integrated safety acceptance.
