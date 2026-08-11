# Tasklist

- [x] 最新ログから逸走開始点を特定
- [x] 通常OvertakeLine/body-clear handoffと別事象であることを確認
- [x] 直接制御のPass admissionを厳密化
- [x] 曲線追従を保持する操舵補正boundsを実装
- [x] 単体テスト追加・更新
- [x] `multi_purpose_mpc_ros` build/test

## Definition of Done

- corridor membership単独では直接Passへ入らない
- 高速曲線で前周期操舵が低速車回避の対称clipによりゼロ側へ縮まない
- 壁stop時は操舵・速度とも停止指令になる
- 既存を含む対象テストが成功する
- 次回`make dev2`でWP278付近のログに、即時`Shift -> Pass`と4 m級逸脱が再発しない

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25 suites）
- 対象パッケージ集計: 979 tests、0 errors、0 failures、0 skipped
- 実走確認はユーザー実施予定
