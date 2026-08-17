# Tasklist

- [x] 現HEADとPro案の実装済み範囲を照合
- [x] 要件・設計・DoDを記録
- [x] Frenet線形化の経路曲率と入力曲率を分離
- [x] RTI-SQP減衰更新helperと単体テストを追加
- [x] MpcProblemの再線形化metadataを追加
- [x] first-feasibleを保持する2-pass solveを実装
- [x] RTI-SQP telemetryを追加
- [x] package build/test
- [x] 今回分だけcommit

## Verification

- `make autoware-build`: success
- `colcon test --packages-select multi_purpose_mpc_ros`: 28/28 CTest success,
  1267 tests / 0 failures（`joycon_contract_guard` の古いbuild artifact警告のみ）
- `ctest -R ^test_mpcc_progress$ --output-on-failure`: success
