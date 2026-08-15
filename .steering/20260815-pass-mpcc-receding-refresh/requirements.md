# Requirements

## 目的

ShiftOut から rear-clear までの追い越し実行を、使い切り型の Frenet DP 経路から、現在状態を起点に繰り返し更新する receding-horizon 実行へ進める。

## 観測された問題

- 最新走行では DP 経路の残距離が 0 m になっても、refresh_count=0 のまま Pass が継続した。
- rolling candidate は生成されているが、`reference=0, horizon=0` として pending のまま昇格しない。
- Pass 中の速度指令は 11.11 m/s であり、抜きっぷり低下の主因は縦速度 cap ではなく横経路の失効と再計画不成立である。
- robust 推奨余裕で補正が必要な候補を、物理 hard clearance を満たすか再検証せず棄却している。

## 必須要件

1. rolling DP 候補を現在の実行経路へ滑らかに接続する。
2. 近傍の実行 prefix は直前の feasible path を保持し、下流だけを新候補へ移行する。
3. robust 推奨余裕で不成立でも、既存の物理 wall clearance で再検証する。
4. 現在 horizon 全体が wall・target・横加速度の hard 条件を満たした場合だけ atomic に昇格する。
5. pending 候補は現在の feasible path を破壊しない。
6. target ID、pass side、V2X continuity、hard fault の既存条件を維持する。

## 制約

- ROS 2 topic/service、評価インターフェースは変更しない。
- Recovery、Reverse、Return の責務は変更しない。
- wall/target の hard clearance は縮小しない。
- ユーザーの `aichallenge/result-summary.json` は変更・コミットしない。
