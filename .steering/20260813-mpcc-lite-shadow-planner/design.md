# Design

## 1. Shadow candidate

既存plannerが生成したMissionから次の共通指標を抽出する。

- rear-clear予測時間・距離
- rear-clearまでの最低速度
- 最小壁余裕
- 最小対車両surface余裕
- 最大横加速度
- 必要横移動量

候補種別は `Left`、`Right`、`CurrentSideHold`、`Return` とする。
Returnはrear-clear確認とReturn corridor成立をhard feasibilityとして扱う。

## 2. MPCC-lite score

hard feasibilityを先に判定し、成立候補だけを以下で順位付けする。

- rear-clearが早い・短いほど加点
- 最低速度、壁余裕、対車両余裕が大きいほど加点
- 横移動、横加速度が大きいほど減点
- 現在のFSM戦術には小さな継続biasを与え、shadow推奨のチャタリングを抑える

これは制御最適化そのものではなく、Phase 2で使用する戦術枝のshadow評価器である。

## 3. 周期とlast feasible

- candidate評価: 8 Hz
- 定期ログ: 1 Hz
- 推奨branchまたはFSM一致/不一致が変化した場合は即時ログ
- 現周期で解がない場合、0.5秒以内のlast-feasible結果を診断用に保持

last feasibleはログ専用であり、既存Missionや制御出力へ書き戻さない。

## 4. ログ

`Overtake MPCC-lite shadow`として次を記録する。

- target、phase、FSM branch、shadow branch、一致/不一致
- 各branchのfeasible、score、rear-clear時間/距離、最低速度
- 最小wall/target余裕、最大横加速度、横移動
- current resultかlast-feasible holdか

## 変更範囲

- `v2x_overtake_core.hpp/.cpp`: typed shadow evaluator
- `mpc_controller_cpp.cpp`: candidate変換、8 Hz実行、周期ログ、設定読込
- `config.yaml`, `config_for_cloud.yaml`: shadow-only設定
- `test_v2x_overtake_core.cpp`: score、hard reject、tie、Returnの単体テスト
