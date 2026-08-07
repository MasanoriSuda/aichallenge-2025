# Tasklist

- [x] 最新ログと既存 progress extension を照合する
- [x] requirements/design を作成する
- [x] 動的完遂予算による local window 延長を実装する
- [x] 設定・起動ログ・診断ログを追加する
- [x] pure policy test を追加する
- [x] format/build/test を実行する
- [x] 実走確認項目を記録する

## 実走確認項目

- `count=1/1` 後も安全かつ完遂可能なら `dynamic completion extension` が出ること
- 対象ケースで `local distance limit` ではなく `Pass -> Return -> Idle` へ進むこと
- absolute 距離/時間を超えて再延長しないこと
- 予測重複、車体重複、壁異常、EmergencyBrake、solver recovery は従来どおり中止されること

## Verification

- `make autoware-build`: 25 packages build successful
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 targets、0 failures
- `colcon test-result --verbose`: 901 tests、0 errors、0 failures
- 既存の `build/joycon_contract_guard/package.xml` 欠損警告は継続（対象packageの失敗ではない）
