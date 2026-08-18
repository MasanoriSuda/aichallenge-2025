# Results

## 実装結果

- 静的壁探索を、preferred offsetごとの単一区間返却から全collision-free component返却へ
  局所リファクタした。
- live controller cache keyからMission依存の`lower/upper/preferred`を除外した。
- headingを0.01 rad bucket化し、bucket内の最大回転差を包含するよう車体外接半径からfootprint
  guardを追加した。clearanceは切り上げ、sample stepは切り下げて保守側へ量子化する。
- scalar bounds全体を一度scanし、現在のlocal trust regionとpreferred offsetはcache取得後に選ぶ。
- cache全消去による周期的thrashをやめ、上限到達時は1 entryずつ入れ替える。
- 1秒周期でrequests/hits/misses/hit rate/scans/scanned poses/entriesを出力する。
- Pass latch後に現在車体と予測sweepが非重複なら、相手制約をtracking MPC hard boundへ
  二重適用しない。MPCC trajectory生成、物理profile再検証、wall bound、SafetyBrakeは維持した。

## 静的検証

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 28/28成功
- `colcon test-result --verbose`: 1298 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 成功

## 次回試走で見る値

比較元は`output/20260818-091414`。

- `Physical wall envelope cache`のhit rate。Pass/ShiftOut継続中は高率になること。
- control callback overrun: 74.73回/分から明確に低下すること。
- Pass callback平均: 11.89 msから低下すること。
- Pass OSQP平均: 3.68 ms、平均iteration 1263から低下すること。
- `pass_release=1`中にsolver fallback、actual overlap、EmergencyBrakeが増えないこと。
- `Pass -> Return -> Idle`完遂が0回から増えること。

動的効果確認は未実施。次回`make dev2`ログで判定する。
