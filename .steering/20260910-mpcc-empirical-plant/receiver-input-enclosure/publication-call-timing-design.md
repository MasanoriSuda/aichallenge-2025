# Publication call timing, baseline fe5f16a2

Continue autonomous M1–M6. Dev2-r48 is failed, not moving acceptance. D1dispatch1156
has nominal15.384999677, callback ROS15.399999655, before15.404999655,
after15.414999655, deadline15.409999677. The18.840074ms whole wall callback
fits25ms but the original certified bracket crosses its deadline by4.999978ms.
Both Ready phases now have normal publications after the context producer fix.

The before/after bracket includes strict final guard, raw diagnostic publication
and final command publication. Generation capture is a small shared generation
reference; no solver/hash calculation. No evidence yet assigns the10ms ROS
advance to any one call, scheduling preemption or incoming clock burst. The
control timer is wall-clock based whereas the immutable programme is ROS-time
based; phase alignment is a separate contributor hypothesis. DDS is not proven
root cause. The exact saved rejected bracket must remain rejected.

Add observation-only steady and ROS timestamps after final guard and raw
publication, plus steady timing immediately after final publication. Preserve
original before/after guard, call order, actual ledger, all proof/authority,
25ms window/period,130ms origin and receiver250ms profile. Extend existing
publication log and failure snapshot; do not add an alternate normal authority.
Wall call measurements include scheduling and sampling overhead. They do not
claim internal DDS CPU time. Additional clock samples add measurement overhead.

Bounded r299 saved failure replay will reproduce the original exact source and
current fullproof then before-pass/after-fail and benchmark final guard. Build
and package tests are required before diagnostic dev2-r49. Only after the first
new exact measured failure is saved may the responsible producer be repaired.
Rollback fe5f16a2. No rate, grace, tolerance, hold or authority changes.

R299 rebuilds the exact original input fingerprint, independently accepts current
physical proof in5.963480ms and retains before-pass/after-fail
at the saved actual clocks.10000final guards all accept the original before
clock; mean0.101193us in isolation. This is not a DDS or live
scheduling measurement. Buildr89passes26packages; testsr77passes2532records/
64groups, zero errors/failures/skips; source104pass. Next diagnostic dev2-r49.
