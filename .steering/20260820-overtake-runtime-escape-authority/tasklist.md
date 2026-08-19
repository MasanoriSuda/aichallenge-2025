# Tasklist

- [x] 最新ログから authority loss の順序を特定する
- [x] pre-contact escape を committed Pass geometry owner に追加する
- [x] wall-bounded escape の前周期検証結果を continuity bridge へ渡す
- [x] DynamicMissionWait cross-side assessment lease を追加する
- [x] runtime-failover trace を拡張する
- [x] unit test を追加する
- [x] package build / test を実行する
- [ ] 動的試走で効果確認する（ユーザー実施）

## Verification

- `make autoware-build`: success
- `ctest --output-on-failure`: 29/29 passed
- focused tests:
  - `test_overtake_decision_trace`: passed
  - `test_v2x_overtake_core`: passed (776/776)
