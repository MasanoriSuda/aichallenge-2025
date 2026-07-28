# Requirements

## Purpose

一度、対象車・通過側・車両膨張済みcorridorを実行可能と判定した追い越しを、
一時的なSafetyBrakeや入口専用の曲線完遂判定で忘れず、物理的に実行可能な限り
rear-clearまで完遂する。

## Observed behavior

提出環境ログでは次の負のループが発生した。

1. `ShiftOut`中にwall / lateral-acceleration guardで`Recovery`へ移る。
2. 近距離になり`SafetyBrake`へ移ると、OvertakeLineのtarget、side、corridorを全消去する。
3. 停止後の再開判定が現在の実速度差を要求するため、Follow状態から追い越し速度差を作れない。
4. 空きcorridorが復帰しても追い越しを再開せず、対象車が動くまで待つ。

また、通常追い越しの入口preflightはbase road boundsで最終横目標を検証している一方、
実行時は車両膨張済みcandidate corridorと異なる目標をlockし、目標値を周期ごとにslewしていた。
そのため、入口で成立したShiftOutが実行中に長期化し、将来のwall clampで失敗し得た。

停止車local pathでは、既にpass corridor内にいる場合にdirect controllerを起動せず、
移動中の別車両をcorridor検証から除外していた。

## Functional requirements

- `ShiftOut`または`Pass`中のSafetyBrakeはPass Missionを破棄せず、一時停止する。
- 一時停止中は横lineや追い越し速度を出力せず、SafetyBrake / front-riskを最優先する。
- 危険解除後は同じtargetとsideを優先し、最新のV2X、wall、candidate corridor、
  lateral-acceleration条件を再検証してから`ShiftOut`を再開する。
- 旧sideが物理的に不成立なら、設定で許可された範囲で反対側を再評価する。
- 追い越し完遂距離判定は新規entryだけを制限し、commit済みmissionの再開を
  「すでに実速度差があること」で循環的に拒否しない。
- targetがrear-clearになるまでPass Missionのtarget ownershipを保持する。
- 通常entryでpreflightした車両膨張済みcorridor内の最終横目標を、そのまま実行lineへlockする。
- preflightと実行は同じ固定横目標とShiftOut距離profileを使う。
- 停止車local pathは、停止車をmaneuver triggerとしつつ、同じlookahead内の移動車も
  blockerとしてcorridor検証に含める。
- 停止車local pathが成立し、egoが既にpass corridor内ならdirect controllerを
  `Pass` phaseから即時起動する。
- gap未成立の停止車candidateだけでは、既存Pass Missionを破棄しない。
- direct controller実行中にmoving blocker等でlive corridorが不成立になった場合は、
  旧targetへ走り続けず停止し、再成立した最新corridor targetで再開する。

## Safety requirements

- Emergency / SafetyBrakeの速度0、front-risk、actual footprint wall contact、
  static-wall physical infeasibility、solver/odometry fail-safeを緩和しない。
- 明示禁止WP、target position jump、course-progress discontinuity、V2X timeout、
  両側の物理corridor不成立では追い越し再開を許可しない。
- Pass Missionの継続保証は「連続して物理的に成立するcorridorがある場合」に限る。
- ROS 2 topic、service、message、launch、評価結果schemaを変更しない。
- 変更は`aichallenge_submit/multi_purpose_mpc_ros`内に閉じる。

## Acceptance criteria

- committed mission中のSafetyBrakeでtarget ID、side、validated goalを保持する。
- SafetyBrake中はOvertakeLine outputがinactiveである。
- 危険解除後、同じtargetの有効corridorがあれば停止速度からでも同じmissionを再開する。
- commit済みmissionはentry-only completion guardだけを理由に拒否されない。
- candidate corridorとtarget separationの共通区間がないsideはcommit前に棄却される。
- 実行lineが使うfixed goalはpreflight resultと一致する。
- 既にcorridor内の停止車回避でもdirect `Pass`制御が開始される。
- low-speed corridorは移動中の他車を障害物として含む。
- low-speed direct controlはlive corridor不成立中に速度0となる。
- pure policyの回帰テストと対象package build/testが成功する。
