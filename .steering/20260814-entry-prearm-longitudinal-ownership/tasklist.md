# Tasklist

- [x] `20260814-072352`の入口デッドロックを特定
- [x] pre-arm speed reference floorをcoreへ追加
- [x] same-target validation leaseによる縦pre-arm継続を追加
- [x] 単体テストを追加
- [x] 対象packageをbuild/test
- [x] 次回試走の確認項目を記録

## 検証結果

- `make autoware-build`: 成功、25 packages
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 targets成功
- `colcon test-result --verbose`: 1105 tests、0 errors、0 failures、0 skipped
- 既存の`build/joycon_contract_guard/package.xml`欠損に関する読取警告は対象外

## 次回試走で見るログ

- `prearm=1`時に`prearm_v`が前車速度＋選択closing speedとなること
- pre-arm中の実速度・相対速度が増え、`entry_rel >= 0.3`を0.3秒維持できること
- 短いcandidate missで`overtake entry leased pre-arm`となり、Follow capへ戻らないこと
- fresh Mission再成立後に`Follow -> Overtake -> ShiftOut`へ移ること
- SafetyBrake、wall contact、横加速度hard faultが増えないこと
