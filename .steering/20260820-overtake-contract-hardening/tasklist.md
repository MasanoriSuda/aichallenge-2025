# Tasklist

- [x] 直近試走ログから契約違反を特定する
- [x] speed window normalization を実装・試験する
- [x] DynamicWait lateral hold authority を実装・試験する
- [x] Pass/runtime wall contract を共通化し trace へ接続する
- [x] 通常 chatter の集約と warning 即時出力を実装・試験する
- [x] package test/build を実行する
- [x] 差分レビュー後にコミットする

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `test_overtake_execution_orchestrator`: 13 tests / 0 failures
- 外部 ROS 2 topic、launch、message、評価schema: 変更なし
