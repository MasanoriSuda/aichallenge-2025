# Tasklist

- [x] `20260819-164659`のPass/SafeSeparation失敗経路を照合
- [x] bounded soft-abort replan leaseをcore/controllerへ追加
- [x] forward-motion stallをcore/controllerへ追加
- [x] target execution floorをconfig/MPCCへ追加
- [x] READMEと起動ログを更新
- [x] core unit testを追加
- [x] Docker内build/testとYAML整合を確認
- [x] ユーザー所有変更を除外してコミット

## Definition of Done

- soft abort後にPass保持WARNをcontrol周期で反復しない。
- 一回のSafeSeparation episodeで再計画リースを再armしない。
- 接触ペナルティ相当の低速・進捗停止を有限時間でhandoffする。
- MPCCのtarget boundは車体実寸に10 cmの実行余裕を加えた値を下回らない。
- build/testが成功し、`aichallenge/result-summary.json`をコミットしない。

## Verification

- `make autoware-build`: 25 packages success
- `colcon test --packages-select multi_purpose_mpc_ros`: 28/28 CTest success
- `colcon test-result --verbose`: 1366 tests, 0 errors, 0 failures
- YAML parse / `git diff --check`: success
- `colcon test-result`には既存の`build/joycon_contract_guard/package.xml`欠落通知が出るが、
  対象packageのテスト結果には影響しない。
