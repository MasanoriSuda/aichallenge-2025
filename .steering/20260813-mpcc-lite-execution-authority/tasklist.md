# Tasklist

- [x] 最新走行の shadow candidate coverage を確認する
- [x] requirements / design を作成する
- [x] 局所 prefix candidate admission を実装する
- [x] MPCC execution authority policy を実装する
- [x] controller から executable Mission を cache / 適用する
- [x] config と起動ログを更新する
- [x] unit test を追加する
- [x] package test / build を実行する
- [x] 実走確認項目を整理する

## 検証結果

- `make autoware-build`: 25 packages成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 1065 tests、error/failure 0
- MPCC追加テスト: 9 tests成功

## 実走確認項目

- `planning_unavailable` の比率が大幅に減ること
- `authority=entry` が新規 ShiftOut で出ること
- authority 適用時の side と frozen Mission side が一致すること
- no-return 後に opposite-side authority が出ないこと
- optimizer miss で即 Follow/失速せず、同じ target の last-feasible が最大 0.5 秒だけ保持されること
- wall/contact/solver hard fault で last-feasible が使われないこと
- Pass -> Return -> Idle 完遂率と rear-clear 時間
