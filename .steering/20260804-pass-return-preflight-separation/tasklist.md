# Task list

- [x] Return phaseの既存実行horizonとguardを確認する
- [x] Initial admissionがfull-path preflightを維持することを確認する
- [x] Pass continuation policyへpath scopeを追加する
- [x] controllerのcommitted Pass preflightをPass-onlyへ変更する
- [x] policy単体テストを追加する
- [x] build/test/diff checkを実行する

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result --verbose`: 834 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 問題なし

## 動的確認項目

- `ShiftOut -> Pass -> Return`の完遂数
- `static full-path preflight`起因のSafeSeparationがactive Pass中に消えること
- `static Pass-continuation preflight`が実際のPass側壁・横加速度不成立を維持すること
- rear-clear後のReturnで壁Recoveryや接触が増えないこと
