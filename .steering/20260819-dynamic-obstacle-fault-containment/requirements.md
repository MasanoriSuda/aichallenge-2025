# Requirements

## 背景

`output/20260819-083421` の6周走行では、動的障害物として認識した低速車に対して
追い越しを開始できている一方、将来の対車離隔と壁境界が両立しないたびに
Missionを破棄し、FollowPrepareまたはRecoveryへ落ちている。

主な観測値は以下。

- 追い越しepisode: 27
- `ShiftOut -> Pass`: 15
- `Return`到達: 3
- 正常な`Return -> Idle`: 2
- Recovery進入: 13
- `physical target separation conflicts with wall bounds`: 13遷移、54ログ
- `SafeSeparation aborted: invalid input`による`Pass -> Recovery`: 1
- rear-clear後の`Return -> Recovery`: 1

## 目的

新しい戦術や状態を追加せず、既存MPCC/FSMの縮退動作を修正する。

1. 数秒先のTarget/壁競合だけで、現在clearな実行prefixを早期破棄しない。
2. SafeSeparation入力の一時欠落をhard faultとして即Recoveryへ送らない。
3. rear-clear済みReturnの最適化失敗を、追い越し失敗としてRecoveryへ送らない。

## 制約

- 機能凍結を維持し、新しい戦術、FSM状態、車両モデル、solverは追加しない。
- 実車体の壁接触、壁サンプル欠落、非回復接触、EmergencyBrakeはfail-closedを維持する。
- ROS 2 topic/service/message、評価インターフェースを変更しない。
- `aichallenge/result-summary.json`の既存変更を変更・コミットしない。

## Definition of Done

- Target-only conflict用holdの時間・距離が、将来競合位置までの安全prefix長を反映する。
- runtime wall preplan warningだけでは、物理検証済みPass prefixの進捗延長を終了しない。
- SafeSeparationの`InvalidInput`はsoft failureとして再計画経路へ流れる。
- rear-clear済みReturnはtarget continuityに依存せずlast-feasibleを再検証できる。
- rear-clear済みReturnで物理接触がない場合、最適化失敗だけでRecoveryへ入らない。
- `multi_purpose_mpc_ros`の単体試験とビルドが成功する。
