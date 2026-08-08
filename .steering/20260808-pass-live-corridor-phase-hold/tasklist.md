# Task list

- [x] `output/20260808-213349`の失敗区間を特定する
- [x] ShiftOutのhold期限がPassへ持ち越されることをコードとログで照合する
- [x] Passフェーズ用hold基準時刻resolverを実装する
- [x] controllerへ統合し、診断ログを追加する
- [x] 単体テストを追加する
- [x] `docs/spec/mpc-integration.md`へ規約を反映する
- [x] 対象packageのテストを実行する
- [x] `make autoware-build`を実行する

## Definition of Done

- 物理的に非重複なPassは、ShiftOutで消費したlive-corridor hold期限を引き継がない。
- 同じPass中はholdが自分自身を延長しない。
- hard safety/continuity guardの条件は変更されない。
- 対象テストとビルドが成功する。

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 tests）
- `colcon test-result --verbose`: 923 tests、0 errors、0 failures、0 skipped
  - 既存の`build/joycon_contract_guard/package.xml`欠損に対するskip診断は出たが、
    対象packageのテスト結果とコマンド終了コードは正常だった。

