# Results (in progress)

## Causal isolation

Baseline S1 replay reproduces all architecture rejections, including Automatic
at coupled obstacle row 496 / stage 3 and Follow StayBehind at effective-progress
row 497 / stage 4. Exit code 4 means no accepted bundle, not a tool failure.
S2's actual warm-start nonlinear oracle passes unchanged wall/dynamic/terminal
proofs (progress 16.5672 m, velocity 3.29412 m/s).

The time-only comparison keeps the source fingerprint 13608911548693048044 and
creates candidate 17803104293055415054. Original A rejects at stage 3; aligned X
passes the solver and every exact physical/terminal certificate, reaching
16.5671 m at 3.29412 m/s with lateral reserve 0.0871511 m. Observed solve times
were 109.320 ms / 26.416 ms, single offline measurements, not runtime percentiles.

Root cause: CurrentTargetTube consumes the inactive legacy dynamics clock in
ordinary normal intents, while seven-state inputs use semantic progress time.
The frozen moving target therefore advances ten times too slowly in the QP.
The earliest broken invariant precedes candidate generation and solver rows.
The final Emergency is the legitimate fail-closed response, not the root cause.
Generation/horizon-size checks failed to detect this semantic mismatch.

The production change consumes progress_stage_dt_sec directly. An empty
metadata vector yields an incomplete tube rather than a legacy-clock fallback.
StageCorridor ownership, ReplayWorld, authority, margins, costs, solver options
and all runtime configuration are unchanged. No origin, acceleration or
projection change was needed to certify this frozen case. Those separate
approximations are not declared equivalent beyond the measured scene.

## Verification so far

- Baseline `make autoware-build`: 25 packages passed.
- Four new comparison tests: passed (variable semantic dt, stationary target,
  lateral clamp/truncation, non-affine/unsealed rejection, immutable source).
- Deletion gate before repair: 1 failure at legacy clock consumption.
- All source authority contracts after repair: 102 passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 59/59 CTest targets
  passed; package-only `colcon test-result`: 2301 tests, 0 errors/failures/skips.
- Original S1/S2 YAML and wall hashes checked unchanged after all replays.
- Historical registry references reconciled: 7 dangling IDs removed; historical
  entries without independent replay remain replay_ready=false.

Logs: output/mpcc-20260907-baseline-comparison.log,
output/mpcc-20260907-baseline-oracle.log,
output/mpcc-20260907-semantic-time-comparison.log,
output/mpcc-20260907-time-audit-tests.log. See manifest.json and timing.md.

Repair `make autoware-build`: 25 packages passed (4 min 19 s).
Installed controller SHA-256: b3cc4965dcfd4df3120f0b96434855ed223caf647ccf5e65ec46e6f607ba6ef0.
Bounded dev2 completed and was stopped for diagnosis. Dynamic and integrated acceptance remain pending.

## Bounded dev2 (one attempted run)

Run `20260907-mpcc-semantic-time-dev2`, same installed binary/config/map in dev2-manifest.json.
D1 joined ShiftOut -> Pass at decision 1495, then lost the retained Pass terminal
contingency at decision 4166 (after acceptance at 4165). SafetyBrake followed at
4177, then FollowPrepare timed out. No Return completion was observed in this
attempt. The next independent slice is `../20260907-mpcc-pass-terminal-audit/`.
This unlimited development run was manually stopped, so absent finish/result JSON
is not recorded as an official evaluation result.

Both MCAPs were read. D1/D2 control receive intervals: means 24.995/24.990 ms,
p95 29.254/27.916 ms, p99 32.627/30.136 ms, maxima 42.300/43.833 ms, no gap >50 ms.
Source-stamp intervals differ (including repeated simulator stamps); do not treat
them as receive gaps. Callback summary maxima were 57.037/27.516 ms with 8/2
overrun cycles. These are diagnostic observations, not formal campaign acceptance.

The dev scenario already auto-started. An extra admin start publication had no
matching subscriber; that waiting one-off command was stopped without changing
contracts or the running race. All diagnostic driving containers were then stopped
through make down and bags finalized.
Host `pre-commit` is unavailable (command not found); `git diff --check` and
registry JSON/reference validation passed. This is not recorded as a hook pass.
