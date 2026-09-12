# Required origin in the initial corridor sampling lattice

2026-09-12 JST. Baseline/rollback70475357. Autonomous M4–M6 remains authorized.
This is the continuation of the same completion task, with a bounded initial
geometry producer repair; integrated race acceptance remains open.

Single-r4 final1022 is captured by the repaired observer and replayed exactly in
R503: source539/job1018/index1, matching source/domain/history/last actual send,
wall6 at14.314999687. It is an actual final loss, not intermediate883. The raw
velocity source13.929999688 is0.120788544m/s, whereas final telemetry1022 is0.07m/s.
The original runner's >0.1m/s trigger therefore does not terminate it. It was
explicitly interrupted for analysis after stationary source rejection persisted;
no Start/laps or840s-deadline result is claimed. Original DLL/user JSON restored.

R504 four native endpoint-input samples stay clear, but they are neither an
exhaustive population nor actual receiver traces. R506 native and compiled full
current enclosures both reject the original cell631/189 at14.314999687. R508
separately checks all existing branch corner images and still rejects at the
same time. No proof partition, map, tolerance or timing change is promoted.

The separate stopped source592 at15.169999660 solves its QP, then its artifact
rejects initial lateral0.8620005388 versus map upper0.8609313490. The artifact's
existing lateral tolerance remains unchanged. R505 A reproduces this artifact
failure; B/C/D/G are target-unavailable and inconclusive for target-free Cruise,
not a physical infeasibility result. R507 finds the same physical initial pose
clear with original footprint/0.2m clearance/0.05m margin; a diagnostic boundary
search gives upper0.9095533659 after the original1mm guard. That search is not
promoted. R509 only includes the immutable origin after its existing ±1mm guard
support passes the same full footprint check; the exact saved QP/primal then
builds a production artifact. The independent external-primal physical chain
passes in both variants and does not mask the original artifact failure.

Producer defect: the regular50mm scan can end a clear component at the sample
just before the origin, then select_lateral_clear_interval returns that nearest
component with preferred_lateral_contained=false. The controller ignored that
flag, so a valid native initial pose was assigned incompatible physical metadata.
The stage-zero equality remains correct; later artifact construction rejects it.

Add the origin-minus-guard, midpoint and origin-plus-guard as explicit scan
samples. Keep every original lattice sample, all occupied/Unknown checks, scalar
bounds, footprint and guard; no finer global sample step or guard/tolerance
reduction. Sorted additional samples cannot bridge a blocked regular sample.
Require the selected component to contain the semantic initial state before
binding it as current physical support. Remove acceptance of a merely nearest
component. Default callers retain their original lattice and results. The optional
C++ helper argument requires a complete rebuild of its package consumers; ROS
contracts are unchanged.

Regression uses exact source592 pose and the identical existing r75 map crop
(full original map SHA190563b7...). Two scalar windows expose lattice phase
sensitivity without changing state/world. Occupied origins and anchor support
outside scalar bounds remain rejected. Before/after native comparison, source
contracts, build119/package106, original unsafe current replay and same-source
full production pipeline precede a fresh standard single-r5. No observation,
offline source witness or local test completes the original M4–M6 work.

Build119 all26 packages and package106 all2602 records pass with zero errors,
failures or skips; source106 and native65 pass. R511 uses built full source
solver/artifact/physical/Stop libraries: original592 rejects, minimally corrected
initial support accepts. Original scalar scan bounds were not serialized; this
is not a reconstruction of exact live output bounds. R512 retains the exact
original current1022 rejection. Review confirms default caller lattice arithmetic,
original guard/sample step and final artifact/current checks remain unchanged.
[Sealed run, comparisons and validation](required-origin-corridor-evidence.json).
Next fresh committed standard single-r5; all M4–M6 remain open.
