# Tasklist

- [x] 最新 P2 ログで solver crawl から復帰不能位置までの遷移を確認する
- [x] 既存 crawl 判定と Recovery rejoin 許容値を確認する
- [x] crawl request に path/footprint safety 入力を追加する
- [x] controller から追従誤差・既存閾値・現在 footprint を渡す
- [x] 安全範囲内、閾値超過、非有限値、静的接触の単体テストを追加する
- [x] `make autoware-build` を実行する
- [x] package test を実行する
- [x] 差分をレビューし、実走確認条件を記録する

## Definition of Done

- 事象開始時の `e_y=0.628 m` / `e_psi=-0.740 rad` では crawl が不成立になる。
- 安全な Cruise・clear footprint・許容誤差内では従来どおり crawl が成立する。
- crawl 不成立時は既存の solver fallback 強制停止が有効になる。
- package test とビルドが成功する。

## 実行結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- 対象 test-result: 677 tests, 0 errors, 0 failures, 0 skipped
- 実走確認: 未実施。`make dev2` でユーザー確認予定。
