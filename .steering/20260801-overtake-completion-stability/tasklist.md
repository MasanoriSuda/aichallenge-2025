# Tasklist

- [x] 最新結果JSONとOvertakeLine遷移を集計
- [x] SafetyBrake、wall Recovery、solver連続失敗のコード経路を特定
- [x] 要件と変更しない安全条件を定義
- [x] unseparated closing reserveをcoreへ追加
- [x] predicted overlap confirmationをfront danger/capで共有
- [x] Entry preflightをgap planner lookahead全域へ拡張
- [x] Recovery中SafetyBrakeでmission stateを保持
- [x] wall marginを0.15 mへ調整
- [x] 単体テストを追加
- [x] package testとbuildを実行
- [x] 差分レビューと試走確認項目を確定

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25）
- `colcon test-result --verbose`: 785 tests、0 errors、0 failures
  - workspace内の既存`build/joycon_contract_guard/package.xml`欠損警告は出たが、
    今回対象packageのtest failureはなし。
- `git diff --check`: 成功

## 試走確認項目

- 起動ログに`unseparated_front_reserve=0.25 m`、wall clearance 0.15 mが出ること。
- OvertakeLine debugで`unseparated_reserve=1`の区間がtarget速度を下回らず、
  `moving front inside hard center distance`へ直行しないこと。
- 一時的なpredicted overlapが0.25秒未満ならfront capとfront dangerが同じ状態を保つこと。
- `Recovery -> Idle, reason=safety brake`が発生せず、代わりにRecovery hold/releaseが出ること。
- 10回以上の試行で`Return -> Idle`完遂数、SafetyBrake合計時間、Recovery数、
  wall/crash penalty、MPC連続失敗最大周期を変更前と比較すること。
