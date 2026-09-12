# Stopped source805: exact cell and native feasibility comparison

2026-09-13. Numerical baselinefdee0b8f, same single-r11source805/world/model/input
limits. No raw candidate executes. Source804uses the same physical observation.

R653locates the first rejected occupied cell476325at the front-left footprint.
Initial largest separating gap0.03188881397m falls to-0.00069058662m at native
stage71/time0.3582711317s after about0.042m displacement. Resolved footprint
front1.615/rear0.51/left-right0.968/margin0.05m includes original clearance once.
Geometric perturbations are diagnostics, not feasible steering or reverse controls.

R654stops at a diagnostic sample-index mismatch; it performs no optimization.
R655corrects the comparison to include native sample0. Both old quarter-drive
witnesses match all363native samples: velocity exactly equal, world position
within1.4e-11m and yaw within3.2e-15rad. These numerical checks do not validate
nominal model accuracy against simulator contact dynamics.

R655uses the original20stages/1.796537451s, free acceleration/steering-rate inputs,
zero virtual speed, original cumulative steering limits, and original footprint
SAT separation from3841nonfree cells within6m at each native endpoint. The optimizer
is only a search: every final candidate passes through the original complete
native wall/dynamic/terminal Stop checker. Three initializations and80iterations
per initialization were declared before running; total581.78s within600s cap.

| Initialization | Final forward displacement | Original full oracle |
|---|---:|---|
| quarter-drive, negative steering | 0.044957m | accepted;14iterations |
| quarter-drive, positive steering | 0.044410m | wallstage358rejected;80iteration limit |
| original A solved inputs | 1.020338m | semantic steering sequence rejected;80iteration limit |

The accepted trajectory is only4.5cm, below the predeclared0.1m diagnostic useful
witness threshold. No useful restart was found. Failed local optimization is
Unknown, not forward infeasibility. Do not repeat seed/iteration sweeps without a
changed structural hypothesis or immutable world. Normal source-wall repair and
measurement/contact error remain open independently of the Recovery session fix.

[Sealed inputs, outputs and hashes](recovery-session-rejoin-evidence.json).
[Separate Recovery/Rejoin design](recovery-session-rejoin-design.md).
