# Recovery vehicle count at the participant entry

2026-09-13. Baseline07897bed. Continue the authorized MPCC completion and local
commits; no push. Fix the missing production launch binding, not the strict
clearance policy or its numerical parameters.

single-r13/R671:4236callbacks,361actual normal publications through899/source509,
no out-of-window normal send. One startup callback485takes30.772436ms. Ready-only;
no Start/laps/Rejoin. Confirmed Recovery now advances through the original waits
into WaitForClear then SafeStop; no confirmation cancellation loop and no V2X
force override. Runtime and protected artifacts were restored on exit.

The startup log reports simulation=1,declared_vehicle_count=0,self_filter=excluded.
Make dev declares AIC_VEHICLE_COUNT=1 and the single empty publisher starts, but
reference.launch.xml includes aichallenge_submit_launch/launch/control/mpc.launch.xml,
which constructs mpc_controller_cpp directly without recovery_vehicle_count. The
previous R669tested only the package standalone Python launch. It passed its narrow
scope but missed the actual production entry; it did not prove standard propagation.

R674follows the participant reference's actual include, parses it with the ROS XML
frontend, expands arguments and evaluates the Node parameter dictionary without
starting nodes. All5cases fail with KeyError recovery_vehicle_count: absent,1,2,4,
and explicit3override. Add the same environment/default0 argument and node binding
to the real MPC XML entry; register this regression in aichallenge_submit_launch's
CMake test and declare its ROS launch test dependencies. Keep participant entry,
topics, Domain separation and all clearance/session/gear/normal authority unchanged.
The additive contract is already documented in participant-interface.md.

Acceptance: all5real-entry cases pass with integer values; make autoware-build;
colcon test/test-result for the changed launch package; source/quality/link checks;
fixed-source single-r14observes declared1and complete current V2X; dev2validates
actual2declaration, native peers, no empty publisher and both Domains. This is an
entry-wiring repair; physical source planning, load-dependent timing and all race/
submission acceptance remain open. Rollback07897bed. Do not repeat unchanged973
native or2633controller tests for this launch-only fix.

R675corrected entry:5passed. Build129:26packages/6.62s with unchanged frozen
inputs. Package116: Summary: 12 tests, 0 errors, 0 failures, 0 skipped. Local entry repair is verified; live propagation
and moving Recovery remain the next checks. [Evidence](recovery-launch-entry-evidence.json).
