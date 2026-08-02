# Tasklist

- [x] 現行の動的mission corridorとentry preflightを確認する
- [x] 動的corridorの検証終端距離を指定可能にする
- [x] 候補選択をROS非依存coreへ追加する
- [x] controllerで横目標・ShiftOut距離の複数候補を評価する
- [x] 選択ShiftOut距離をmissionへ固定し実行処理で共用する
- [x] core単体テストを追加する
- [x] build/test/diff checkを実施する
- [x] 動的`make dev2`確認項目を記録する

## 動的確認項目

- `Overtake mission candidate selected`のgoal/shift距離
- Idle -> ShiftOut -> Passの成功数
- Pass -> Return -> Idleの完遂数
- entry rejection理由と候補試行数
- SafetyBrake、wall Recovery、接触回数
