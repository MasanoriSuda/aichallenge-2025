# Task list

## 調査・設計

- [x] 最新P1/P2ログを別事象として分類する。
- [x] Behavior、OvertakeLine、低速direct control、stuck recoveryの所有権順を確認する。
- [x] P1の証拠なしReverseへ至る再現経路を確定する。
- [x] 修正範囲を低速回避の所有権と再計画に限定する。

## 実装

- [x] commit済みOvertakeLineを低速回避候補から保護する。
- [x] active低速回避でも反対側static/V2X preflightを許可する。
- [x] active側切替時にdirect control phaseをShiftへ戻す。
- [x] 状態変化ログを追加する。
- [x] 単体テストを追加・更新する。

## 検証

- [x] 対象packageの単体テストを実行する。
- [x] `make autoware-build`を実行する。
- [x] `git diff --check`を実行する。

### 検証結果

- `make autoware-build`: 成功、25 packages finished。対象packageのstderrは既存のsetuptools非推奨警告のみ。
- `colcon test --packages-select multi_purpose_mpc_ros --ctest-args -R test_v2x_overtake_core`: 成功。
- `colcon test-result --verbose`: 870 tests、0 errors、0 failures、0 skipped。別packageの古い`build/joycon_contract_guard/package.xml`欠損警告は残存。
- `git diff --check`: 成功。

## 動的確認（オペレーター）

- [ ] `make dev2`で停止・極低速車へ接近する。
- [ ] commit済み`ShiftOut/Pass`が`stopped vehicle bypass owns target`で破棄されないことを確認する。
- [ ] 低速direct controlの選択側が塞がった場合、成立する反対側へ切り替わることを確認する。
- [ ] `live vehicle corridor unavailable -> evidence_free_qualified -> Reverse`が再発しないことを確認する。
- [ ] 壁接触、車両接触、solver failure、Recovery回数を比較する。

## Definition of Done

- [x] P0のplanner所有権競合をコード上で解消した。
- [x] active低速回避に安全な反対側再計画経路がある。
- [x] 既存のhard safety gateを維持した。
- [x] 静的検証結果を記録した。
- [ ] 動的確認結果を記録した。
