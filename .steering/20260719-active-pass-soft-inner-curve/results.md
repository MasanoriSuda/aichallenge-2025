# Results

## 結論

`v2x_overtake_continue_inner_soft_curve_enabled=true` のA/Bは不採用とする。対象の
WP148はsoft-inner区間であると同時に、MPC horizon内にhard hairpinが入る区間だった。
そのため追加したsoft-inner継続許可よりhard curve guardが優先され、従来と同じ位置で
Overtakeを中断した。現行configは`false`へ戻し、policy実装と診断ログだけを残す。

## 有効run

- output: `output/20260719-191752`
- D2: WP130で`Follow -> Overtake`、front distance 8.59 m
- D2: WP148で`Overtake -> Follow`、front distance 3.68 m
- 中断時: `gap=1, soft_curve=1, hard_curve=1, inner_pass=1, continue=0`
- 中断理由: `overtake hard curve blocked`
- 2周目もD2はWP130で追い越しを開始し、WP148で同じhard guardにより中断した
- D1もWP133から追い越し、WP148で`hard_curve=1`により中断した

## 走行結果

手動停止前に3台とも1周を記録し、全車停止は再現しなかった。

| 車両 | 1周目 |
|---|---:|
| D3 | 126.230 s |
| D2 | 131.803 s |
| D1 | 138.289 s |

D2のOSQP maximum-iterationsログの大半はスタート待機中の速度0 m/sで発生した。
走行中にも一時的なsolver fallbackはあったが復帰し、全車が2周目へ入った。

## 無効run

- `output/20260719-191607`: MPCバイナリのリンク完了前に起動したため旧実装を使用
- `output/20260719-190954`: 診断項目追加前。WP148の拒否理由を`before-curve`と誤表示

## 次候補

さらに攻める場合はinner-soft flagではなく、hard curveをMPC horizon全域で即中断条件に
する設計を見直す必要がある。hard境界までの残距離とOvertakeLineのPass/Return進捗から
「境界手前で追い越し完了可能か」を判定する別A/Bとし、hard threshold自体は変更しない。
