# Requirements

## 背景

`608f573` の `make dev2` 走行 (`output/20260813-105959`) では、D1 の13回の追い越しがすべて Pass へ到達した一方、正常な `Pass -> Return -> Idle` は2回だった。

- `Pass -> FollowPrepare`: 7回
- `Pass -> Recovery`: 4回
- receding-horizon 有効デバッグサンプル: 54
- legacy fallback デバッグサンプル: 16
- MPC solve failure: 0

再計画器は動作しているが、その可解性が Behavior 所有権と SafeSeparation の継続判断へ伝わらず、前車速度への固定、短い観測欠落、壁警告後の再捕捉で Mission が分断されている。

## 目的

可解な receding-horizon 経路を持つ ShiftOut/Pass が、入口候補の再検索失敗や一時的な V2X/最適化欠落で中断されないようにする。

## 必須要件

1. 最新の可解な receding-horizon 経路に、短時間かつ同一 Mission generation/side 限定の実行 lease を持たせる。
2. lease 中は、入口候補の再検索失敗だけで Behavior を Follow/Cruise へ落とさない。
3. Pass の車体・壁・予測が直前まで安全だった短い観測欠落では、SafeSeparation を `invalid input` で即 Recovery にしない。
4. SafeSeparation 中も lease が有効なら、前車速度固定ではなく既存の full-speed forward escape を利用できるようにする。
5. optimizer の一時的失敗時は、現在の制約で再検証できた直前可解経路だけを短時間保持する。
6. actual wall contact/margin violation、EmergencyBrake、solver recovery、position jump、明示禁止 waypoint は従来どおり hard fault とする。
7. runtime wall warning からの Return は同じ target の即時 Pass 再捕捉を禁止する。
8. fallback/lease の理由を過剰にならない既存周期ログへ追加する。

## 非目標

- 本格的な joint lateral/longitudinal MPCC への置換
- actual wall/contact guard の撤廃
- Recovery/Reverse 全体の変更
- ROS 2 topic/service/message 契約の変更

