# Tasklist

- [x] 現行候補生成と実行gateを確認する
- [x] Progressive Entryの設定とcandidate metadataを追加する
- [x] body-clear候補を実行Missionへ段階昇格する
- [x] Complete Mission優先の左右選択を維持する
- [x] tactical revalidationの早すぎるReturnを抑制する
- [x] core単体テストを追加する
- [x] package build/testを実行する
- [x] 動的確認項目を記録する

## Definition of Done

- 全Mission候補がない場合でも、body-clear可能な初回候補がOvertakeLineへ渡る。
- 全Mission候補が存在すればProgressive候補より優先される。
- active Mission replanへProgressive候補を混入させない。
- 2 m前方に対象が残る状態ではtactical Returnしない。
- 既存および追加単体テストと対象package buildが成功する。

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 518 tests passed
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result --verbose`: 1057 tests、0 errors、0 failures
  - unrelatedな古い`joycon_contract_guard/package.xml`欠損を集計時に警告するが、テスト失敗はない。

## make dev2で確認するログ

- `progressive entry candidate selected`が、従来の`candidate search rejected`区間で出ること。
- `OvertakeLine: Idle -> ShiftOut`後にbody-clearへ進むこと。
- Complete Missionがある区間では`complete mission candidate selected`が選ばれること。
- `SafeSeparation target clear ahead; speed-preserving Return`がtarget_s < 4.0 mで出ないこと。
- wall margin violation、actual body conflict、横加速度超過が増えていないこと。
- Progressive Entry後のrolling replan、Pass -> Return -> Idle完遂率と最低速度。
