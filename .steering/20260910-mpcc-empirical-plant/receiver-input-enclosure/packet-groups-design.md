# Causal acceleration packet groups

2026-09-11 JST, rollback 7b7fef37. dev2-r17 keeps certified Stop but does not
restart; D1/D2 callback p99 is 50.8731018/27.11461325 ms, with 5815/41 adjacent
overrun pairs. The run was interrupted externally after persistent rest and
observed timing failure; neither race deadline nor race acceptance was reached.

Exact D1 decision 1020 at ROS 15.099999662, proposed source 619, reproduces
AppliedProgramUnavailable/StateBoundRejected at 15.339999662. The nominal
proof succeeds. Source 619 is the proposed Stop, not automatically the actual
published source (the contemporaneous trace publishes Stop 544). Historical
1019/source 544 is explicitly a different source, not substituted predecessor
evidence for 619.

Producer: mpcc_applied_input_prediction.cpp channel_bounds collapses all causal
acceleration values to one convex interval before hybrid propagation. Actual
values in the problematic window are -3 and approximately +1.329596. Their
hull introduces unsent tiny positive acceleration into the moving branch;
near rest, interval widening yields u lower -0.0074326155519167355, outside the
original 0.00741003100948554 tolerance. This is a numerical representation
loss, not measured reverse motion or a demonstrated physical infeasibility.

Frozen native tools r78/r79 propagate negative/zero/positive packet hulls
separately, retain all causal windows, exact clocks, steering range, scalar
model, original wall and full CA1 peers. This gives u lower 0, full rest,
wall clear and minimum peer clearance +0.6283457957656704 m. All 299008 native
scalar values are enclosed. Numerical time is about 7 ms; this does not prove
25 ms live acceptance. The diagnostic has no execution authority or Follow
claim. Sources, archives and libraries are frozen under tool-r78/frozen-inputs.

Same-world A/B/C/D nominal comparison succeeds for persistent A, stateless
left B, rough left C and bounded offline left D. Right arms reject; complete
rest feasibility also succeeds. These comparisons do not include the new
applied certificate and cannot execute. They locate the current rejection
after nominal solving, not in Mission feasibility.

Repair: return sign-separated acceleration hulls from the existing packet
selection pass and propagate their union through the shared hybrid map before
merging the existing body-mode bins. Every actual packet, equal-epoch value,
channel combination and repeated tail remains admitted. Preserve the overall
min/max fields as bounding diagnostics; they are not the propagation domain.
Steering remains independently convex. Preserve the single-interval primitive
and its continuous-range regression. No constants, tolerances, bounds, solver
budgets, rates or authority requirements change. The provenance hash still
binds the same complete source and program; this is a tighter enclosure of
the same physical input contract, not a new packet or clock contract.

Validation: first fail an exact observation/program test on the old library;
check same-epoch/zero/small-positive/negative packets and independent steering
combinations; retain native full-interval primitive checks; run package build
and tests, original saved-world certificate/materialization/join replay and
four library source worlds. Update library oracles to sample the declared
union and independently check every admissible packet is in it. Recompile
all native drivers after the AppliedInputBounds ABI change. Then fresh dev2
must establish actual stop/restart and timing. If normal rejection persists,
capture the original failed candidate before Stop fallback replaces its
diagnostics; current terminal snapshot helper omits AppliedProgramUnavailable.

Remaining M4–M6 and model/contact/transport guarantees are open. No margin
relaxation, observation clipping or late authority bypass is part of this fix.

## Implemented and validated

Old exact regression fails with minimum u -0.00751549 against the same tolerance.
Current union passes. r80exact1020/source619 now accepts the required certificate,
production adapter, Stop materialization and join, with rest15.429999662 and
minimumpeer+0.62849003239852341m,6.636435ms initial retained evaluation. The
diagnostic r79used the old longer horizon; do not compare those sample counts
as identical work. Historical1019/source544 remains an invalid predecessor
association, even though replaying its own request separately succeeds.

Original r16signed Stop795/source124 still accepts through its explicit upstream
intent helper and joins with unchanged peer margin. Current continuous-domain
checks retain2017600input/956800hybrid scalar values. Four original source worlds
pass374616native values, independent channel selections and every causal packet
inclusion check; prediction7.06–10.04ms. Buildr41=26packages/4m58s; testsr33=
2430records/62groups/zeroerrors-failures-skips. Added test macro braces remove
the new compiler warning; target rebuilt and all22modeltests passed again.
The first sandbox socket attempt never ran a test; approved escalation exposed
the old failure. Logs and original failed run remain preserved.

Code review found no ROS interface, model, source identity or authority change.
The internal AppliedInputBounds ABI grew; all consumers and native drivers were
rebuilt. The full-interval primitive remains available and verified. Fresh
committed dev2-r18is next; actual restart, live timing and M4–M6 remain open.
See [validation manifest](packet-groups-evidence.json).
