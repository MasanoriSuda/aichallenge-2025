# Tasklist

- [x] 直近走行のrolling replan効果と未完遂原因を確認する
- [x] Pass MPCCの最小変更範囲と安全境界を文書化する
- [x] paused replacementのphase-aware実行modeを追加する
- [x] Pass originのsame-side候補を現在状態からPass継続としてcommitする
- [x] 累積Pass距離、横補正、rear-clear、Return距離を保持する
- [x] 単体テストを追加する
- [x] ビルド・テスト・差分検査を実行する

## Definition of Done

- Pass由来の同側rolling replacementが`FollowPrepare -> Pass`になる。
- ShiftOut由来およびcross-side replacementは`ShiftOut`を維持する。
- Pass継続時に新規ShiftOutの速度policyへ戻らない。
- `make autoware-build`と対象package testが成功する。

## Dynamic acceptance checks

次回`make dev2`では以下を確認する。

- `Pass -> FollowPrepare -> Pass`が発生する。
- 同側replacement logが`mode=pass-continuation`となる。
- `dynamic wait selected fresh same-side Mission`による14 m級ShiftOut再開が消える。
- `Pass -> Return -> Idle`が発生する。
- wall/physical Recoveryとcross-side no-return違反が増えない。

## Verification

- `make autoware-build`: successful（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: successful
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  1080 tests, 0 errors, 0 failures, 0 skipped
- `git diff --check`: clean
