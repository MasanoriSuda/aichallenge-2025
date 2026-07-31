# Results

## 実装結果

- solver failure crawl に既存 Recovery rejoin envelope の横偏差・方位偏差 gate を追加した。
- 生 odometry pose の現在 footprint が静的地図上で clear の場合だけ crawl を許可した。
- unsafe 時はcrawlを不成立にし、既存のsolver fallback強制停止とStuck Recovery判定へ渡した。
- P2実測値 `e_y=0.628 m` / `e_psi=-0.740 rad` を回帰テストへ追加した。
- unsafe理由を1秒throttleのログへ追加した。

## 検証

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros/test_results --verbose`:
  677 tests, 0 errors, 0 failures, 0 skipped

## 実走確認項目

`make dev2` の再走ではP2ヘアピン付近で次を確認する。

1. `MPC solver fail-operational crawl blocked by path safety` が出る。
2. 従来の `MPC solver fail-operational crawl entered` が同じ逸脱状態で出ない。
3. `e_y` が約0.63 mから4 m超へ拡大する前に車両が停止する。
4. 壁根拠を伴うsolver fallbackが継続した場合、Stuck Recoveryがコース近傍から開始する。
5. `maneuver_direction_unknown -> SAFE_STOP` の反復へ入らず前進復帰する。

実走結果は未確認。
