# Adaptive Contact Escape and Rejoin Solver Grace Requirements

作成日: 2026-07-18
状態: Completed（dev3受け入れは部分成功）

## 背景

`output/20260718-011435`では、D3がWP282付近の連続solver failureからReverse-only Recoveryへ
入ったが、現在footprint 16 contactに対するStraight / Left / Right候補が最初の0.05 mで
contactを増やし、`maneuver_direction_unknown`でSafeStopした。現行Left / Rightは
`reverse_steering_angle_rad=0.25`の1種類だけで、中間操舵角を評価していない。

同runのD1は2.059 m後退してLowSpeedRejoinへ入った直後、D2はDrive復帰時にsolver fallbackが
残っていたため、即座に`solver_unsafe`でSafeStopした。Recovery中のMPC履歴reset直後には
短いsolver再初期化時間を許容する必要がある。

## 要求

1. Side / Mixed contactの初回step候補はStraightに加え、左右の操舵角を0から既存最大角まで決定的に分割して評価する。
2. 候補採用条件は既存のcontact非増加、局所連続、終端contact減少、static swept footprintを維持する。
3. 選択した操舵角をepisode中に固定し、後続周期で最大角へ変化させない。
4. static候補がない場合はForwardや強制Reverseへ切り替えずSafeStopする。
5. LowSpeedRejoin中のsolver fallbackは最大1.0秒、Driveのまま停止保持して復旧を待つ。
6. 待機中はLowSpeedRejoin駆動、alignment完了、Normal復帰を行わない。
7. solverが復旧すればLowSpeedRejoinを再開し、1.0秒以内に復旧しなければ`solver_unsafe`でSafeStopする。
8. stopping reserveの実測差で2.0 m直前にstep上限へ達しないよう、最大3.0 mの総距離上限を維持したままstep上限を10とする。
9. ROS topic、service、message、Domain、評価JSON契約は変更しない。
10. simulation-onlyと既存V2X / gear / 距離 / 速度 / 時間gateを維持する。
11. 成功したRecoveryのstep / attempt予算を、次の独立したRecovery episodeへ持ち越さない。

## 2025 AWSIM向け暫定値

- Side / Mixed steering sample数: 5
- 最大Reverse steering: 0.25 rad
- サンプル角: 0.05、0.10、0.15、0.20、0.25 radの左右
- rejoin solver recovery timeout: 1.0秒
- 最大escape step: 10（総後退距離上限3.0 mは維持）

## Definition of Done

- 操舵サンプル生成、選択角保持、solver待機・復旧・timeoutをunit testする。
- `make autoware-build`と対象unit testが成功する。
- `make dev3`でD1〜D3の候補操舵角、static contact、gear、移動距離、LowSpeedRejoin、最終状態を記録する。
- contact悪化中のReverseCreep、solver fallback中のLowSpeedRejoin駆動、後方不完全時の後退をFailとする。
- 全停止が解消しない場合は次の拒否理由をログで特定する。
