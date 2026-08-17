# Tasklist

- [x] 最新走行の左右候補と選択経路を照合する
- [x] 要件・設計を記録する
- [x] 直線入口の物理幅優先を実装する
- [x] Progressive Entry 限定の実寸離隔 fallback を実装する
- [x] preflight 棄却診断を追加する
- [x] unit test と package build/test を実行する
- [x] 変更をコミットする

## Definition of Done

- 直線で実行可能な左右候補に十分な開き幅差がある場合、広い側を選択する。
- lookahead curve label だけでは直線入口の side を固定しない。
- ロバスト離隔 fallback は Progressive Entry 系に限定される。
- 完全 Mission、壁 hard bound、車体実寸離隔は従来どおり保持される。
- 棄却理由をログから判別できる。

## Verification

- `test_v2x_overtake_core`: 追加2件を含め通過
- `colcon test --packages-select multi_purpose_mpc_ros`: 28 test targets、1283 tests、失敗0
- `make autoware-build`: 25 packages build成功
