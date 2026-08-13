# Tasklist

- [x] `20260813-200341` の異常終了時刻を確認
- [x] move後参照を特定
- [x] move先の`validated_horizon`参照へ修正
- [x] `multi_purpose_mpc_ros`をビルド（`make autoware-build`: 25 packages成功）
- [x] 単体テストを実行（25/25 test groups成功）
- [ ] `make dev2`で2周以上の動的確認（ユーザー実施）

## Definition of Done

- 対象packageのビルドと単体テストが成功する。
- 動的確認で`mpc_controller_cpp`のexit code -11が再発しない。
