# Design

## Scope

新しいROS nodeやtopicは追加せず、既存の`V2XBehavior`、`OvertakeLineState`、
`V2XGapPlanner`の責務境界を維持する。`FollowPrepare`は現行コードで宣言済みだが
未使用のため、SafetyBrakeで一時停止したPass Missionの状態として使用する。

## Persistent Pass Mission

Pass Missionのcommit条件は次の組で表す。

- stable target vehicle ID
- selected pass side
- preflight validated fixed lateral goal
- optional inter-vehicle corridor boundary IDs

`ShiftOut` / `Pass`中にSafetyBrakeが発生した場合は、上記を保持したまま
`FollowPrepare`へ移る。`FollowPrepare`は制御lineを出力しないため、
SafetyBrakeと通常Followの縦制御が優先される。

危険解除後はbehavior層がlocked targetを再投影し、左右gapとentry preflightを再評価する。
旧sideが成立すれば同じsideを、成立しなければ`overtake_try_both_sides`の範囲で反対側を選ぶ。
`Overtake`が再選択された周期に`FollowPrepare -> ShiftOut`へ遷移し、phase距離を現在位置から
再計測する。

再検証結果がstart-grid車間corridorではなく通常corridorなら、旧boundary vehicle IDと
inter-vehicle flagを同じ周期でclearし、古いrear-clear契約を次missionへ持ち越さない。
gap成立前のlow-speed candidateだけではこのownershipを破棄せず、feasibleな
`LowSpeedAvoidance`またはactive direct controlが実際に横計画を所有した場合だけhandoffする。

一時停止中にtargetがrear-clearならReturn、position jump / discontinuity / timeout /
explicit forbidden WPならRecoveryへ移る。actual wall contactは従来どおりRecoveryを優先する。

wall / corridor等でRecoveryへ入った場合も、距離または横復帰による通常完了で、
target continuityが有効かつactual wall contact、solver recovery、explicit forbidden WPが
なければ、状態を消去せず`FollowPrepare`へ移す。stall / timeoutは従来どおりmissionを解除する。

## Completion guard ownership

hard-curveまでに追い越しを完遂できるかのdistance estimateは新規entry admissionである。
`line_committed=true`のmissionには再適用しない。commit後の継続可否は、最新のgap preflight、
wall footprint、target continuity、Emergency、forbidden WPで判定する。

これにより、SafetyBrakeでego速度が0付近になった後に
`ego_speed - target_speed >= threshold`を先に要求する循環を除く。

## Entry/execution consistency

通常side assessmentではcandidate gapのactive intervalを全horizonで交差し、
base road boundsとの共通区間をpreflightへ渡す。target separationを満たす固定goalが
共通区間内にない場合はcommit前に棄却する。

preflightが返した`goal_ey`を`overtake_corridor_center_ey`として保存し、
`OvertakeLineState.fixed_pass_corridor_goal_ey`へそのままlockする。
ShiftOutの横軌道自体は既存のdistance-based smoothstepを使うため、commit時にendpointを
fixed goalへ設定し、周期単位のendpoint slewは重ねない。Return / Recoveryの中心復帰slewは維持する。

## Stopped-vehicle handoff

停止車の存在だけをLowSpeedAvoidanceのtriggerに使う。local-path corridorのside評価には、
同じlookahead内の全active V2X vehicleをinflated blockerとして含める。

local pathがfeasibleなら、egoがcorridor外では`Shift`、既にcorridor内では`Pass`として
direct controllerを起動する。これにより「corridor成立済みなので起動条件を通らない」
逆転を解消する。wall guardとSafetyBrakeはdirect controllerより高い優先度を維持する。

direct controller中も毎周期local pathを再評価する。moving blocker等で不成立になった周期は
速度・操舵を0にし、旧targetへ走り続けない。再成立時は最新の`pass_target_ey`へ更新して再開する。

## Interface impact

- ROS 2 topic / service / message: 変更なし
- launch / parameter key: 変更なし
- evaluation result schema: 変更なし
- participant package内部のFSM・planner・pure policyのみ変更

## Runtime verification

次回`make dev2`ではP1について以下を確認する。

- `ShiftOut/Pass -> FollowPrepare`がSafetyBrake時に一度だけ出る。
- Brake解除後に同一targetで`FollowPrepare -> ShiftOut`となる。
- 通常Recovery完了後も対象が前方または並走中なら`Recovery -> FollowPrepare`となる。
- pause中にOvertakeLine debugのactive targetがMPCへ適用されない。
- entry logのvalidated goalとShiftOut debugの`corridor_goal`が一致する。
- `ShiftOut`が設定距離を大きく超えて長期化しない。
- 停止車corridor内から`Low-speed pass shift control entered`が`Pass`速度で出る。
- moving blockerでlive corridorが閉じた間は`live vehicle corridor unavailable`で停止し、
  corridor復帰後に再開する。
- wall contact、target jump、Emergency時には従来どおり停止またはRecoveryとなる。
