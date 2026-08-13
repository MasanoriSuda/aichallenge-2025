# Tasklist

- [x] 最新試走のtarget/wall Recovery経路を特定する
- [x] preferred/hard境界の責務を設計する
- [x] elastic clearance設定を追加する
- [x] 初期target境界にhard-wall retryを追加する
- [x] post-validationを物理target hard境界へ分離する
- [x] last-feasibleをhard execution境界で再検証する
- [x] 単体テストを追加・更新する
- [x] package build/testを実行する
- [x] 試走確認項目を記録する

## Static verification

- `docker compose run -T --rm --no-deps autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test groups成功
- `colcon test-result --verbose`: 1096 tests、0 errors、0 failures、0 skipped
- elastic target boundsについてpreferred成立、hard-wall retry、無効時fail-closedの3ケースを追加

## Dynamic verification

- `physical target separation conflicts with wall bounds`が、hard壁＋物理target境界の真の不成立時だけ出ること
- `optimized horizon escaped target separation bounds`が減ること
- debugのhard-wall／soft-clearance縮退が増えても、wall contactとSafetyBrakeが増えないこと
- `ShiftOut -> Pass -> Return -> Idle`の完遂率が上がること
- `Pass -> Recovery`が減ること
