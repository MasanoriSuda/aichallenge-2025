# Tasklist

- [x] 最新走行のSafetyBrake・車間・状態遷移を集計する
- [x] 既存closing reserveの作動条件を確認する
- [x] requirements/designを作成する
- [x] closing reserveを横離隔基準へ局所リファクタする
- [x] Mission entryの動的必要車間を実装する
- [x] Follow/entry距離設定を整合させる
- [x] 単体テストを追加する
- [x] ビルドする
- [x] コミットする

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25）
- `colcon test-result --verbose`: 1172 tests、0 errors、0 failures
