# Results

## Root cause

`output/20260830-145407` の失敗では、live Pass identityの要求が
`overtake_line_output.active && stage_corridor.active` に従属していた。
現在側のstage corridor不成立時に、再計画へ必要なtarget、generation、phase、sideまで
消失し、same-epoch sibling branchを評価するseven-state problem scopeが閉じていた。

## Change

- `CanonicalExecutionIdentityRequest` を
  `overtake_execution_requested` という戦術identity専用契約へ改名した。
- callerはShiftOut/Pass/Returnというlive phaseからidentityを要求する。
- stage corridor、物理wall proof、dynamic proof、publisher tokenは従来どおり独立した
  problem/certificate契約として維持した。
- orchestrator unit testとsource contract testで、legacy corridor availabilityが
  identity requestへ混入しないことを固定した。

## Static validation

- `make autoware-build`: 成功、25 packages。
- `colcon test --packages-select multi_purpose_mpc_ros`: 59/59 tests passed。
- `colcon test-result --verbose`: 2240 tests、0 errors、0 failures。
- `joycon_contract_guard/package.xml` の既存stale build artifact警告は出たが、対象packageの
  test failureではない。

## Dynamic validation

`make dev2` により `output/20260830-150910` を取得した。

- Overtake episode: 3。
- `Idle -> ShiftOut -> Pass`: 3回成立。
- Pass中のseven-state normal command: 37件。
- Pass中のfailsafe command: 10件。
- `proposed=pass ... proposed_world=intent-mismatch`: 97件。これらは旧ShiftOut
  artifactから新Pass artifactがjoinするまでのhandoffであり、本修正対象の
  corridor-loss後identity消失と同一とは分類しない。
- branch pipelineの`store=accepted`: 68件。
- `stage_corridor.active=0 && phase in {ShiftOut, Pass, Return}`: 0件。

したがって、live Passからseven-state normal authorityを作れることは確認したが、
旧failureのstage-corridor-loss条件は今回のrunで発生していない。根本契約はunit testと
source contractで固定済みだが、同じ動的条件での直接的な再現防止確認は未取得である。

## Independent remaining failure

3 episodeはいずれも最終的に `committed pass longitudinal progress stalled` へ遷移した。
Pass中には別契約の `proposed_world=progress-lift-rejected` が42件あり、normal authorityが
断続的にEmergency Stopへ切り替わった。これはidentity erasureではない。

次Sliceでは新しいlease、grace、fallback、tolerance調整を加えず、
physical progress、lifted progress、expected thetaの生成元と時刻基準を追跡し、
`progress-lift-rejected` がmodel mismatch、measurement timing、または証明範囲のどれに
由来するかをfailure snapshotから確定する。
