# Tasklist

- [x] 最新走行の Pass 離脱理由を集計する
- [x] closing=0 候補と実速度の予測不整合を特定する
- [x] 要件・設計を記録する
- [x] 予測速度選択を純粋関数へ切り出す
- [x] Frenet-DP 候補生成と atomic promotion へ適用する
- [x] unit test と package build/test を実行する
- [x] 変更をコミットする

## Definition of Done

- 計画速度が現在速度を下回っても target encounter を遅く見積もらない。
- rolling candidate は現在の運動量での physical target separation を通過しない限り実行権限を得ない。
- robust clearance と wall hard guard の既存設定を変更しない。
- 既存テストと追加テストが通る。

## Verification

- `make autoware-build`: 25 packages build 成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 28 test targets、1285 tests、失敗 0
