# Requirements

## Objective

Race MPCC の Entry 前候補評価が、まだ生成されていない locked target を要求して全結果を破棄する循環依存を解消する。

## Requirements

- Idle / Follow / Entry 前は、現在選択中の V2X 観測を provenance として使用する。
- Mission 採用後は、同一 target の provenance を `Observed` から `Locked` へ昇格できる。
- `Locked` から `Observed` への退行、target ID 不一致、時刻・generation 退行は fail closed とする。
- 非同期 tactical result の不採用理由を一意な reason code として定期ログへ出す。
- 片側 branch の失敗時に、可行な反対側または Hold を維持する既存の縮退動作を壊さない。
- ROS topic / service / message、評価結果 schema は変更しない。
- `aichallenge/result-summary.json` の既存変更には触れない。

## Acceptance criteria

- `invalid-expected` が Entry 前の通常状態で継続しない。
- Race MPCC shadow で左右 branch の `attempted` が観測できる。
- 非同期結果ログから破棄理由を一意に判別できる。
- provenance の `Observed -> Locked` 昇格と退行拒否を単体テストする。
- 対象 package の build / test が成功する。
