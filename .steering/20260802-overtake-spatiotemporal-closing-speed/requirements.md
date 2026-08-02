# Requirements

## 目的

既存の追い越し候補探索へclosing speed軸を追加し、採用した横経路と
closing speed上限を同じmissionとしてShiftOutからbody-clearまで維持する。
横へ出る途中で速度方針が設定最大値へ戻ったり、候補評価時と実行時の速度が
食い違ったりすることで、追い越しがFollow相当へ戻る現象を減らす。

## 変更範囲

- 既存の`side × 横目標 × ShiftOut距離`候補へclosing speed候補を追加する
- 既存の最小・最大closing speed設定から、最小・中間・最大の候補を生成する
- body-clear deadlineを候補ごとのclosing speedで評価する
- 選択したclosing speedをBehavior出力とOvertakeLine mission状態へ保存する
- ShiftOutおよび未解除Passの速度参照でmissionのclosing speed上限を使用する
- deadline成立済みの固定ShiftOutは通常の候補再評価だけでBehavior ownershipを失わない
- 候補選択・mission entry・既存debugログへ選択速度を追加する
- ROS非依存core単体テストを追加する

## 制約

- 制御周期40 Hzを変更しない
- `a_max: 1.0 m/s^2`を変更しない
- 壁、実footprint、SafetyBrake、solverのhard guardを無効化しない
- ROS topic/service/message契約を変更しない
- 既存のShiftOut距離・横目標候補探索を置き換えない
- start-grid breakoutの専用速度方針は変更しない

## Definition of Done

- 同じ横候補に対して複数closing speedがdeadline評価される
- 選択closing speedがmission状態へ固定される
- 実行中の速度計算が固定値を上限として使う
- 安全側のadaptive/reserve処理は固定値より低い速度へ制限できる
- emergency、禁止WP、target不連続、solver recoveryではShiftOut ownershipが解除される
- core単体テストとpackage buildが成功する
