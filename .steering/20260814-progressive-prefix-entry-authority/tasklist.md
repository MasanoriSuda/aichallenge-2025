# Tasklist

- [x] 最新走行のentry authority不整合を特定
- [x] new-entry progressive prefix admissionをcoreへ追加
- [x] MPCC-lite entry authorityへprefixを接続
- [x] fresh current-prefix leaseと`has_executable_mission`を接続
- [x] commit window内のsetup-only加速を禁止して縦横handoffを結合
- [x] 単体テストを追加・更新
- [x] 対象packageをbuild/test
- [x] 次回試走の確認項目を記録

## 検証結果

- `make autoware-build`: 成功、25 packages
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 targets成功
- `colcon test-result --verbose`: 1105 tests、0 errors、0 failures、0 skipped
- 既存の`build/joycon_contract_guard/package.xml`欠損に関する読取警告は対象外

## 次回試走で見るログ

- Idle時のMPCC-liteログが`prefix=1/1/admitted`かつ`authority=entry`になること
- `Follow -> Overtake -> Idle -> ShiftOut`が同じfresh prefixで連続すること
- 15 m以内でprefixが消えた場合、`overtake entry leased pre-arm`のまま直進加速しないこと
- `ShiftOut -> Pass`後はrolling replanが同側補正または有意な反対側候補を選べること
- SafetyBrake回数、wall/contact hard fault、entry side chatterが増えないこと
