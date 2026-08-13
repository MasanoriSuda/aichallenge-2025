# Tasklist

- [x] 最新走行と現行HEADのauthority範囲を照合する
- [x] requirements / designを作成する
- [x] active same-side authorityを実装する
- [x] target-bound projection repairとlast-feasible fallbackを実装する
- [x] near-field locked-target continuityを実装する
- [x] config / 起動ログを更新する
- [x] unit testを追加する
- [x] package test / buildを実行する
- [x] 実走確認項目を整理する

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25 / 25 test成功
- `colcon test-result --verbose`: 1093 tests、0 errors、0 failures、0 skipped
- 既存build treeの`joycon_contract_guard/package.xml`欠落警告は残るが、対象packageのtest resultには影響なし

## 実走確認項目

- `authority=replace`が同側の新鮮なcomplete Missionにだけ出ること
- same-side replacement後もtarget IDとsideが維持されること
- same-side replacement後に解除済みfront capが再適用されないこと
- `optimized horizon escaped target separation bounds`が減ること
- target-bound repairでwall/contact違反が増えないこと
- course projectionの一時欠落時にlocked target距離が`inf`へ飛ばないこと
- position jump、遠方別枝、EmergencyBrakeではnear-field/last-feasibleを使わないこと
- `Pass -> Return -> Idle`完遂率と最悪周回時間
