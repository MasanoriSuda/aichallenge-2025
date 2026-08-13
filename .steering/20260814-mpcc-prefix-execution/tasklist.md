# Tasklist

- [x] `20260814-000850` の不成立経路を整理する
- [x] receding prefix の commit admission を純粋関数化する
- [x] MPCC-lite authority が admitted prefix を反対側置換に使えるようにする
- [x] active Mission replacement に progressive prefix 経路を追加する
- [x] decisive score advantage の即時 commit を追加する
- [x] config とデバッグログを更新する
- [x] 単体テストを追加・更新する
- [x] package build/test を実行する
- [x] 実走確認項目を記載する

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets 成功
- `colcon test-result --verbose`: 1101 tests、0 errors、0 failures
  - 既存の `build/joycon_contract_guard/package.xml` 欠損に関する test-result 読取警告は出るが、対象パッケージの失敗ではない。

## 次回実走で見るログ

- `Overtake MPCC-lite shadow` の `authority=replace`
- `decisive=1` または `cross_pending` が安定時間へ到達すること
- prefix の場合は `prefix=1/1/admitted`
- commit 後に `mode=receding-prefix` が出ること
- 同一追い越しで `Pass -> Return -> Idle` まで到達すること
- actual wall contact、SafeSeparation、collision penalty が増えていないこと
