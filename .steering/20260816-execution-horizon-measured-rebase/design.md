# Design

## 方針

rolling refresh の候補評価を「経路の stitch 方法」と「共通の hard validation」に分ける。

1. `PreserveActivePath`
   - 現在の last-feasible prefix を短距離保持する。
   - 既存どおり滑らかに fresh DP path へ blend する。
2. `MeasuredStateRebase`
   - 通常候補が不成立の場合のみ評価する。
   - 古い prefix/tail を使わず、現在計測 `e_y` を起点に fresh DP path へ blend する。
3. 両者とも以下を同じ関数で評価する。
   - 制御 horizon の完全性
   - planning/physical wall clearance
   - static map wall clamp
   - lateral acceleration
   - time-aligned target physical separation
   - atomic promotion の target continuity / hard fault gate

## 再基準化の許可条件

`resolve_frenet_dp_measured_rebase_retry()` を pure policy として追加する。

- 機能 enabled
- refresh requested
- 通常候補が promote されていない
- target ID が一致
- target progress が連続
- position jump / course progress reject がない
- target prediction が有効
- current body separated または recoverable side contact
- hard fault なし

この policy は「検証を緩める」のではなく、「古い経路を候補生成入力から外して再度同じ検証へ通す」ことだけを許可する。

## 設定

`v2x_overtake_mpcc_frenet_dp_measured_rebase_retry_enabled: true`

ローカル・提出用 config の両方へ同値で追加する。

## ログ

既存の `measured_rebase` を維持し、再基準化候補を評価・採用した場合に 1 とする。候補が不成立でも pending ログから再試行の有無を識別できる。

## 安全性

- measured rebase は hard fault を無視しない。
- promotion 前の全 horizon 検証を省略しない。
- 不成立候補は active path を上書きしない。
- target-bound hold 専用の既存 rebase は包含し、挙動を維持する。
