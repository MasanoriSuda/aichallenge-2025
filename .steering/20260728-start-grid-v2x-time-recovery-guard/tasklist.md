# Task List

- [x] 提出ログの直接原因を特定する。
- [x] ユーザーの既存変更と対象コードを確認する。
- [x] V2X receipt-age pure helperを追加する。
- [x] V2X plannerのfreshness判定をhelperへ統一する。
- [x] Start-grid coordinated Recovery抑止helperを追加する。
- [x] runtimeのcoordinated-stop入口へ抑止条件を適用する。
- [x] 単体テストを追加する。
- [x] 正本仕様を更新する。
- [x] 対象テストを実行する。
- [x] `make autoware-build`を実行する。
- [x] 実走可否と残リスクを記録する。

## 検証結果

- `make autoware-build`: 成功（25 packages）。
- `test_start_grid_grace`: 33 tests passed。
- `test_v2x_overtake_core`: 202 tests passed。
- `colcon test-result`は今回対象を0 errors / 0 failuresとして集計した。一方、過去の
  `build/joycon_contract_guard/package.xml`が存在しない警告は残るが、今回のテスト失敗ではない。
- `make dev3`実走は未実施。提出環境固有のcallback順序差に対する修正なので、次の提出走行で
  `receipt_age=-0.035`前後でも`health=Healthy`、Start-grid中にReverse要求が出ないことを確認する。
