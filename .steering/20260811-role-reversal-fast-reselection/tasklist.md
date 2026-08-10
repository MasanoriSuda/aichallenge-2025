# Tasklist

- [x] 最新実走の失敗遷移を確認する
- [x] rear-clear course-role 評価と dynamic Mission wait を確認する
- [x] role-reversal reserve を実走条件へ合わせる
- [x] SafeSeparation tactical reselection policy を追加する
- [x] dynamic Mission wait 専用の短い期限を追加する
- [x] unit test を追加する
- [x] package build/test を実行する
- [x] 動的確認項目を記録する

## Definition of Done

- rear-clear 直後に内側へ変わる outer Mission が entry で見落とされない
- SafeSeparation 中、相手が十分前方かつ非重複なら fresh alternate を即時再検証できる
- dynamic Mission wait が 4 秒固定にならず、期限後に side lock を解放する
- hard fault と side-by-side crossing は引き続き拒否される
- `multi_purpose_mpc_ros` の build/test が成功する

## Dynamic verification checklist

- `outer pass becomes inside before rear-clear` の発生回数
- `SafeSeparation tactical alternate reselect` の成立/拒否理由
- `dynamic Mission wait reselect timeout` から次 entry までの時間
- `Pass -> Return -> Idle` 完遂率
- crash/wall/Recovery/Reverse 回数

## Static verification result

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 test suites）
- `colcon test-result --verbose`: 992 tests, 0 errors, 0 failures, 0 skipped
- `git diff --check`: 成功
- 実走確認は未実施。上記 Dynamic verification checklist を次回 `make dev2` で確認する
