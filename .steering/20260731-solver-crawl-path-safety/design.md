# Design

## 原因

現行 `resolve_solver_failure_crawl()` は simulation、control、solver fallback、V2X Cruise、
前方車有無、速度上限だけを判定する。最新 P2 走行ではヘアピンで
`e_y=0.628 m`、`e_psi=-0.740 rad`、静的壁接触ありの状態でも 1.0 m/s crawl が成立し、
約 10 秒後に `e_y=-4.603 m` まで逸脱してから Stuck Recovery が開始された。

## 方針

1. `SolverFailureCrawlRequest` に現在の横偏差・方位偏差、各上限、現在 footprint clear
   を追加する。
2. 純粋関数内で有限性、非負上限、絶対誤差、footprint clear を検証する。
3. 上限は新規設定を増やさず、既存
   `stuck_recovery.rejoin.max_lateral_error_m` と
   `max_heading_error_rad` を再利用する。
4. controller は生 odometry pose で現在 footprint を一度 sample し、invalid、地図外、
   接触セルありをすべて unsafe として渡す。
5. unsafe な solver fallback は crawl 非成立となり、既存 `forced_stop_active` で即時制動する。
   既存の solver fallback 継続確認後、壁根拠があれば Stuck Recovery が近い位置で開始する。

## 互換性・安全性

- 変更は参加者 MPC package 内に閉じる。
- 新規 YAML parameter は追加しない。
- 静的地図が利用不能な場合も停止側へ倒す。
- 通常 MPC 成功時の制御、V2X FSM、Recovery の候補選択・速度設定は変更しない。
