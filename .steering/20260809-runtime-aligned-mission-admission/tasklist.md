# Tasklist

- [x] 最新ログと admission/runtime 距離契約を照合する
- [x] runtime continuation reserve の pure policy を追加する
- [x] 初回 Pass 距離へ completion reserve を反映する
- [x] rear-clear course-role 左右比較へ runtime reserve を反映する
- [x] unit test を追加・更新する
- [x] package build/test を実行する
- [x] 動的 `make dev2` 確認項目を記録する

## 検証結果

- `make autoware-build`: 25 packages successful
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 tests passed

## 次回 make dev2 で見る項目

- Mission log の `runtime_reserve=4.00`。
- 左右が成立する場面で、`full_track_transition=0` の側が初回選択されること。
- `SafeSeparation entered: trigger=rear_clear_window` と直後の FollowPrepare が減ること。
- `SafeSeparation abort: reason=local distance limit` が減ること。
- `Pass -> Return -> Idle` 完遂数、wall/contact/SafetyBrake が増えていないこと。
