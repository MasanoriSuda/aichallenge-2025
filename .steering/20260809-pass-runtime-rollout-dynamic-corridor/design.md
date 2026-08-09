# Design

## 1. Runtime completion rollout

`CommittedPassForwardCompletionRequest`へ、共有kinematic rolloutが返した
`prediction_valid`、`rear_clear_feasible`、`required_forward_distance_m`、
`required_completion_time_sec`を渡す。core側で瞬時に目標closing速度へ到達すると仮定した
距離計算は廃止する。

controllerは現在位置から、固定sideの現在goalへ向かう残りlateral ramp、コース速度cap、
`a_max/a_min`、制御遅延、target速度を使って50 ms刻みでrolloutする。

## 2. Same-side dynamic corridor refresh

フルの最適化器追加ではなく、現行のatomic Pass replacementをreceding-horizon化する。

- 対象: Pass中、fresh prediction、target continuity有効、非接触、SafeSeparation未開始。
- side: 現在sideを固定する。
- goal: 現在および予測target lateralから同一sideの安全goalを再計算する。
- 更新: 100--200 ms周期、1回の横変位を小さく制限し、無視できる差はcommitしない。
- 検証: wall bounds、footprint separation、横加速度、rear-clear rollout、atomic generation確認。
- 出力: 既存のlateral replan rampを使い、横goalを段差で変更しない。

これは3--5 knot相当の小さな再計画を時系列にcommitする最小実装であり、sideを毎周期変更するものではない。

## 3. Target Mission total budget

Mission初回freeze時刻を保持し、opponent-side replacementやFollowPrepareを跨いでも再初期化しない。
総時間上限はactive Pass上限と別管理する。

- rear-clear済みかつReturn corridor有効: Return。
- それ以外: 同一target Missionを終了し、既存のhard guardに従うbounded recoveryへ移行。
- ContactContinuation中は即座に通常Followへ戻さず、既存Recoveryへ渡す。

初期値は15秒とし、動的試走で12--15秒をA/Bする。

## 4. Observability

debug logへ以下を追加する。

- runtime rollout valid/feasible、rear-clear距離/時間
- dynamic corridor update回数、旧goal/新goal、更新拒否理由
- Mission総経過時間/上限、budget終了理由

## 5. Compatibility

`/control/command/control_cmd`を含むROS I/O、launch、package境界、result JSONは変更しない。
設定未指定時にも有限かつ保守的な既定値を使う。

