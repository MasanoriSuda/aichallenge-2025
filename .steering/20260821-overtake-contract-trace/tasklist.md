# Tasklist

- [x] 前方安全包絡をpure function化する
- [x] Behavior、entry、runtime closingへ同じ包絡を接続する
- [x] extended branchのeligibilityとboundary fallbackを実装する
- [x] SafeSeparation absolute/local windowの矛盾を修正する
- [x] solver crawl block reasonを構造化する
- [x] 最終decision logへ縦方向契約を追加する
- [x] 単体テストを追加・更新する
- [x] package build/testを実行する
- [x] 差分レビュー後にコミットする

## Verification

- `make autoware-build`: 成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  1434 tests, 0 errors, 0 failures, 0 skipped

## Dynamic verification focus

- `Overtake MPCC-lite async` の `boundary` と左右 `elig` を確認する。
- `Overtake control decision` の `front/safety/protected` と `closing_ref` を確認する。
- solver forced-stop時は `crawl-*` のblock reasonを確認する。
- 壁接触、接触ペナルティ、追い越し完遂率は次回 `make dev2` 走行で確認する。
