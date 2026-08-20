# Tasklist

- [x] 直近ログと現行wall admissionの停止連鎖を照合
- [x] ROS interface契約と既存差分を確認
- [x] wall rejectionのreplan/mismatch判定を追加
- [x] MPCへactive Mission wall rejection handoffを追加
- [x] 決定ログへMission generationとwall契約差を追加
- [x] unit testを追加・更新
- [x] package build/test
- [x] 差分レビューとコミット

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 30/30 test targets成功
- `colcon test-result --verbose`: 1449 tests、0 errors、0 failures
