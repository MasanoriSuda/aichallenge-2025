# Task list

- [x] 最新runと上位車データから接触・失速経路を整理する
- [x] clearance-buffered minimum-motion policyを実装する
- [x] target separation/front-cap/current-overlap設定を整合させる
- [x] transient predicted overlap confirmationをPass continuationへ適用する
- [x] side-by-side forward escapeをSafeSeparationへ追加する
- [x] policy単体テストを追加・更新する
- [x] `make autoware-build`を実行する
- [x] package testと`git diff --check`を実行する

## 静的確認結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 tests）
- `git diff --check`: 成功

## 動的確認項目

- `ShiftOut -> Pass -> Return`完遂数
- Pass中の最小center separationとcurrent overlap confirmation回数
- `safe_sep=1`時のsigned closing speed（横並びでは正になること）
- SafeSeparationによる速度低下時間
- 低速車接触、壁接触、Recovery理由
