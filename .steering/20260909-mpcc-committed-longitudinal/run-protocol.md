# Fixed acceptance after paired longitudinal response repair

First pass full25-package build and all60package CTest groups; run the new
native observer/rollout and actual Stop453/world993 reproof. Then seal source,
untracked new module files, configuration, main/comparison binaries and the
new prediction shared library in each run manifest. A baseline commit plus only
the main executable hash is insufficient for this shared-library change.

Run single first, then dev2. Both use the unchanged physical/simulator settings
and existing declared delay, with only six laps/600s as the run boundary.
Terminate at the first active moving Emergency/Recovery (Ready and Start count),
or after all vehicles finish, or at840s host deadline. Preserve the exact first
failure and the complete run; no unchecked same-settings rerun. Parallel Domain
shutdown begins at an explicit recorded wall time. Preserve/restore user JSON.
No production edit, build or heavy replay while the simulator is active.

Single acceptance: six laps, penalty0, no moving override/Recovery, correct
normal authority identity, current wall/peer proof, callback/publish timing.
Dev2 additionally requires all vehicles and multi-tick moving Stop/restart and
normal peer interaction. A short diagnostic reaching only one interaction is
not a complete race. Missing result-details/penalty is unknown, not zero.
Both Pass/Return signs, dev3/dev4, specific Recovery/Rejoin/async, safety gates
and artifact/baked-image evaluation remain later full-completion tasks.
