# Task list

- [x] 最新走行の authority / wall rejection 時系列を照合する
- [x] ShiftOut speed reference が front-cap release と同時に消える経路を特定する
- [x] Mission 候補へ計画実行速度を保存する
- [x] ShiftOut speed contract の純粋関数と単体テストを追加する
- [x] controller へ speed contract を接続する
- [x] authority conflict と決定ログを追加する
- [x] build / test を実行する
- [x] 差分をレビューしてコミットする

## 検証結果

- `docker compose run -T --rm --no-deps autoware-build`: 成功（25 packages）
- `test_overtake_execution_orchestrator`: 52 / 52 成功
- `test_v2x_overtake_core`: 807 / 807 成功
- `colcon test-result --verbose`: 1484 tests、0 errors、0 failures
  - 既存の `build/joycon_contract_guard/package.xml` 欠損について警告あり
