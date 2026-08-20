# Design

## 事象の切り分け

最新ログでは、DynamicEscape QP の失敗後に
`solver-bounded-continuation -> racing-line` へ戻り、その約0.5秒後から壁証拠、
約1.3秒後に大きな yaw-rate と速度低下が現れた。一方、既存の最終決定ログは
DynamicEscape 中の `wall_min` が `nan` であり、どの経路が壁へ接近したかを
確定できなかった。

## ログモデル

`overtake_execution_orchestrator` に副作用のない次の型を追加する。

- `WallHandoffProbe`: 現在のauthority、制御源、壁近接、操舵、車両状態
- `WallHandoffEvent`: source transition、wall risk、contactの発火理由
- `PredictedPathWallMetrics`: 最終採用予測軌道のfootprint評価結果
- `ChangeAwareWallHandoffTraceEmitter`: 変化検知、2秒監視lease、抑制

イベント判定と文字列化をROS nodeから分離し、単体テスト可能にする。

## 実行時処理

1. 通常の制御計算・公開を完了する。
2. 同じdecision IDの最終authorityとcontrol sourceを取得する。
3. DynamicEscape関与中、solver fallback中、またはそれらからの切替直後2秒だけ
   cheapな現在footprint情報を監視する。
4. source transition、required clearance以下、接触のいずれかでイベントを発火する。
5. 発火時だけ、`current_prediction` の各点からyawを復元してfootprintを壁gridへ
   照合し、最小壁距離、sample index、進行距離、壁方向を算出する。
6. 同一risk状態は0.5秒に1回までとする。

予測軌道が存在しないfallbackでは `prediction=unavailable` とし、古い予測を
使用するsolver bounded continuationでは `retained=1` を明示する。

## ログ例

```text
DynamicEscape wall handoff: decision=3176, trigger=source-transition/wall-risk,
from=dynamic-escape/solver-bounded-continuation,
to=racing-line/mpc-solution, current_wall=Left/0.18m/contact=0,
required=0.40m, prediction=valid/27, path_wall=Right/0.12m@8/4.2m,
pose=..., tracking=..., speed=..., yaw_rate=...,
steering=raw/published/previous/delta
```

## 非対象

- 壁回避ロジックやsolver設定の変更
- AWSIM collision topicの新設
- rosbag topic契約の変更
- Pass runtime corridor不整合の修正
