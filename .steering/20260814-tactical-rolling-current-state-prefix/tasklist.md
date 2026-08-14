# Tasklist

- [x] 直近ログのPass離脱原因とMPCC-lite候補欠落を確認する
- [x] 変更範囲と安全境界を文書化する
- [x] tactical rolling causeを一元化する
- [x] no-return後のsame-side current-state prefix再評価を実装する
- [x] stale holdよりfresh prefixを優先する
- [x] FollowPrepareのownership、速度保持、timeoutをcause間で統一する
- [x] ビルドと関連テストを実行する
- [x] 変更結果と試走時の確認項目を記録する

## Verification

- `make autoware-build`: successful（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: successful
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  1079 tests, 0 errors, 0 failures, 0 skipped
- `git diff --check`: clean

## Dynamic acceptance checks

次回`make dev2`では以下を確認する。

- `Pass -> FollowPrepare`後のshadow logが
  `rolling=1/DynamicMissionWait|TacticalRevalidation|RecoveryRetention`になる。
- same-side候補が成立する場合、`authority=replace_active`と
  `fresh same-side PassPlan replaced`が出て`ShiftOut`へ戻る。
- no-return後にcross-side replacementが発生しない。
- 新候補が成立しない場合も、短いrolling timeout中に通常Follow速度へ
  落ちず、壁・Emergency・solver hard faultは従来どおり優先される。
- `Pass -> Return -> Idle`完遂数が増え、`Pass -> FollowPrepare -> Idle`が減る。
