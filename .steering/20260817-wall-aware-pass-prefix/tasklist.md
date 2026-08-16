# Tasklist

- [x] 最新走行の追い越し遷移、wall warning、target hold を照合する
- [x] 既存 runtime wall center contraction の不成立経路を確認する
- [x] target-bound progress extension を wall-aware にする
- [x] nominal -> physical clearance の center contraction fallback を追加する
- [x] core 単体テストを追加する
- [x] package test/build を実行する
- [x] 差分をレビューしてコミットする

## Static verification

- `make autoware-build`: success (25 packages)
- `colcon test --packages-select multi_purpose_mpc_ros`: 27/27 test targets passed
- `colcon test-result --verbose`: 1238 tests, 0 errors, 0 failures, 0 skipped
  - `build/joycon_contract_guard/package.xml` の既存 stale artifact warning は今回の対象テストに影響なし。
- `git diff --check`: passed

## Review

- target-only の short repair budget は維持し、wall warning 中の Mission-wide progress extension だけを閉じる。
- physical-clearance fallback は current-body separation、selected-side 関係、wall interval、lateral acceleration、full path preflight を全て要求する。
- actual wall contact / hard margin / sample unavailable の優先順位は変更なし。
- topic、service、Domain、launch、result schema の変更なし。
- local `config.yaml` と `result-summary.json` のユーザー変更はコミット対象外。

## Dynamic verification

- [ ] wall warning 後に target-bound progress extension が新規開始しない
- [ ] `runtime wall center contraction accepted` が必要時に発生する
- [ ] `clearance=physical` 使用時も current-body overlap / wall contact が増えない
- [ ] `runtime wall Return suppressed before rear-clear` から wall Recovery へ進む件数が減る
- [ ] `Pass -> Return -> Idle` 完遂率が改善する
