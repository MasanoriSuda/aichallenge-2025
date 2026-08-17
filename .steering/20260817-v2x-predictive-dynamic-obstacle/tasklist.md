# Tasklist

- [x] 最新走行で新authorityの発動範囲を確認する
- [x] 予測発動判定を純粋関数として追加する
- [x] moving targetとslow targetの確認条件を分離する
- [x] start-grid graceを新authorityから除外する
- [x] local/cloud設定を更新する
- [x] 単体テストを追加する
- [x] `test_v2x_overtake_core`を実行する
- [x] `make autoware-build`を実行する
- [x] 変更をコミットする

## Definition of Done

- 15 km/h級車両が相対速度と予測時間に応じて候補化される。
- 同じ距離でも追いつかない車両は候補化されない。
- start-grid専用処理と新authorityが競合しない。
- ユーザーの既存未コミット設定をコミットしない。

## 検証結果

- `make autoware-build`: 25 packages成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 28/28 test targets成功
- 最終ビルド成果物の予測発動テスト: 5/5成功
