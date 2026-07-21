# Task list

- [x] 最新dev3ログで方向遷移と待機時間を確認する
- [x] Reverse方向latchとfallback条件を設計する
- [x] pure core helperとadapter stateを実装する
- [x] 1 km/h停止判定と待機時間を設定する
- [x] unit testと仕様文書を更新する
- [x] package build/testを実行する

## Verification

- `make autoware-build`: success（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 22/22 test targets passed
- `colcon test-result --verbose`: 566 tests、0 errors、0 failures、0 skipped

## dev3 log acceptance

- 起動ログが`stop_entry<=0.28 m/s`、`AWSIM_settle=0.30 s`を表示する。
- AWSIM待機後に`Reverse intent latched`が出る。
- 同一episodeでReverse latch後、明示的なaggressive retryより前に
  `FORWARD_MANEUVER`へ変更されない。
- 後方V2X blockerがいる車両は`WAIT_FOR_CLEAR`を維持し、最後尾が先に動く。
