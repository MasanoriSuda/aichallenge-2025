# Tasklist

- [x] 現行の権限決定箇所とログ不足を確認する
- [x] 実行権限のenum/request/resolutionを実装する
- [x] episode accumulatorを実装する
- [x] controllerの速度・横経路適用をresolution経由へ統合する
- [x] 単一authorityログとepisode終了サマリーを追加する
- [x] unit testを追加する
- [x] `docs/spec/log-design.md`を更新する
- [x] package build / testを実行する
- [x] 変更をコミットする

## Definition of Done

- 同じ周期の横・縦所有者が一行で確認できる
- 権限競合がsilentに通過しない
- 追い越し終了時に一行のepisode summaryが出る
- 既存の速度・横経路適用条件を変えない
- buildと対象testが成功する

## Verification

- `make autoware-build`: success
- `ctest -R "test_overtake_execution_orchestrator|test_overtake_decision_trace"`: 2/2 passed
- package `ctest --output-on-failure`: 30/30 passed
