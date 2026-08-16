# Tasklist

- [x] 最新2走行と前走行を比較し、rolling candidate pending理由を確認する
- [x] raw DPとrolling stitch、実行前validatorの到達可能性モデルを照合する
- [x] steering requirements/designを作成する
- [x] stitch pure policyへ実測状態ベースの到達可能性制約を追加する
- [x] controllerの重複request生成を局所リファクタする
- [x] 診断ログと単体テストを追加する
- [x] build / testを実行する
- [x] 変更をコミットする

## 検証結果

- `make autoware-build`相当: 25 packages successful
- stitch関連gtest: 5/5 passed
- `test_v2x_overtake_core`: 659/659 passed
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 targets、1201 tests、0 failures
