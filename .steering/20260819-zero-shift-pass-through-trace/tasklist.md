# Tasklist

- [x] 最新ログとauthority/runtime分岐を照合する
- [x] 要件・非対象・完了条件を記録する
- [x] validated pass-through authorityをcoreへ追加する
- [x] controllerからbridge検証結果を渡し、planning traceへ記録する
- [x] runtime failover trigger gateとsourceを追加する
- [x] preemptive Mission差し替え結果をtraceへ追加する
- [x] core/trace単体テストを追加・更新する
- [x] 対象packageをbuild/testする
- [x] `git diff --check` と契約差分を確認する
- [x] ユーザー所有ファイルを除外してコミットする

## Verification

- `make autoware-build`: success（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 29/29 test sets pass
- `colcon test-result --verbose`: 1391 tests, 0 errors, 0 failures
  - 既存 `build/joycon_contract_guard/package.xml` 欠損の集約警告は今回の対象外
- 最終build後の対象test再実行: 2/2 pass
- 動的効果確認: 次回 `make dev2` 試走で実施
