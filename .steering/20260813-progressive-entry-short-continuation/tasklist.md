# Tasklist

- [x] 直近2走のProgressive Entry失敗経路を整理する
- [x] entry準備候補と実行候補を分離する
- [x] body-clear deadline余裕0.6秒をProgressive Entryへ適用する
- [x] body-clear後6.0 mの同側preflightを追加する
- [x] static fallback横移動上限を1.8 mへ変更する
- [x] local/cloud設定と起動ログを更新する
- [x] format/build/testを実行する
- [ ] make dev2による動的効果確認

## Definition of Done

- Progressive Entryは0.6秒以上のbody-clear余裕を持つ。
- Progressive Entryはbody-clear後6.0 mの同側経路が成立する。
- static fallback Progressive Entryは横移動1.8 m以下に限定される。
- Complete Missionとentry speed preparationの既存動作を維持する。
- 対象packageのbuild/testが成功する。

## make dev2確認項目

- 前回の0.36秒余裕・1.94 m横移動相当の候補が選ばれないこと。
- `progressive_slack_rejected`または`progressive_short_continuation_rejected`が棄却理由と対応すること。
- `Idle -> ShiftOut`自体が以前の保守状態まで減らないこと。
- Pass -> Return -> Idle完遂率が上がり、wall RecoveryとSafeSeparationが減ること。
- Complete Missionは引き続きProgressive候補より優先されること。

## 静的検証結果

- `git diff --check`: 成功
- `make autoware-build`相当のDocker build: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 targets）
- `colcon test-result --verbose`: 1057 tests、0 errors、0 failures
  - unrelatedな既存`joycon_contract_guard/package.xml`欠損警告は残るが、テスト失敗はない。
