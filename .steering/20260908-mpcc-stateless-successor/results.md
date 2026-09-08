# Local successor repair; Pass observed, integrated acceptance failed

Frozen2677 A/B/C/D/E/F/G/H produced no certified candidate. This is Unknown
without a physical infeasibility certificate. Candidate generation and all
solver/geometry limits are unchanged.

The source deletion regression fails before repair at the Pass/Return draft
boundary. Both worker builders and both draft branches now use the same
reference eligibility: valid frozen Mission or the existing publisher-bound
stateless encounter. The latter requests a fresh solve; it supplies no command.
The Pass worker clears selected_mission when frozen geometry is absent,
avoiding an absent optional dereference or old geometry. Return retains its
preflight path requirements. Existing target validation, serialized predecessor,
solver/physical proof and current-world phase admission remain required.

All104 source contracts pass. Full build and package results are recorded below. Run3 receive-clock control
periods: D1 mean24.994/p95 27.877/p99 30.287/max40.351ms; D2 mean24.993/
p95 27.017/p99 28.858/max36.971ms. No receive gap exceeds50ms. Source-stamp
repeats10 per vehicle are separate from publish cadence. Callback maxima
26.260/25.241ms, one overrun each. Run3 is a failed integration diagnostic,
not six-lap race acceptance or a Stop-publication acceptance.

The same2677 source was additionally replayed with the four Stop cost/support
arms. All four solve numerically but fail exact wall proof (including production
feasibility Stop2671118850129113752). This does not establish absence of every
physical Stop, nor does it justify selecting an uncertified late fallback.
Log: output/20260908-mpcc-stateless-boundary-stop/comparison.log.

Full build:25 packages,4min17s; only existing setuptools deprecation warnings.
Full package:2313 tests, zero errors/failures/skips. Diagnostic run4 starts with
controller SHA-256 de9bb2e0f5877297e93f0bd40fefa750089bf1bc827d5352914583f9069a3486;
source patch, binaries and manifest preserved. Dynamic outcome follows.

Run4 dynamic result: first sibling Bundle796 publishes at1788793847.453847437
and retires Mission geometry. Pass drafts continue afterwards, and ShiftOut
transitions to Pass at1788793856.402930008 with atomic current-world authority.
The Pass-generation repair is observed in simulation. Thirteen sibling
adoptions occur in the episode. Pass then travels about185m while the target
remains ahead; progress watchdog selects Recovery at1788793899.724237010
(decision3465). Return was not requested, so its changed source admission is
not yet dynamically tested. No certified Stop was selected.

The four-minute monitor stopped the run on tactical Recovery. This is partial
lifecycle acceptance, not completed overtake, race acceptance or a Stop test.
No watchdog/weight/speed setting is changed to conceal the new boundary.
