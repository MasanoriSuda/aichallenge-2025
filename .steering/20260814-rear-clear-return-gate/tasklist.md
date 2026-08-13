# Tasklist

- [x] 最新走行ログから早期Return経路を特定
- [x] `RuntimeWallPreplan`へrear-clear gateと同側holdを追加
- [x] MPCC-lite同側置換へscore thresholdとcooldownを追加
- [x] unit test追加・更新
- [x] `multi_purpose_mpc_ros` build/test
- [x] 次回試走の確認項目を記録

## Definition of Done

- rear-clear未成立時に`ReturnToBaseLine`が選ばれないunit testが通る
- rear-clear成立時の既存Return経路が維持される
- 微小score差およびcooldown中の同側置換が抑止される
- 対象パッケージのbuild/testが成功する

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 test targets）
- `colcon test-result --verbose`: 1103 tests、0 errors、0 failures、0 skipped
- 既存の`build/joycon_contract_guard/package.xml`欠損に関する読取警告は対象外

## 次回試走で見るログ

- `runtime wall speed-preserving Return`がrear-clear前に出ないこと
- soft警告が継続した場合は`runtime wall Return suppressed before rear-clear`となること
- hard wall contact/margin時は従来のhard guardが動くこと
- `Overtake MPCC-lite shadow`の`same_admit=0/adv=.../age=...`で微小差更新が抑止されること
- `ShiftOut -> Pass`後、rear-clear成立まで同側を保持し、`Pass -> Return -> Idle`まで完遂すること
- `Return -> Idle`直後のSafetyBrakeが再発しないこと
