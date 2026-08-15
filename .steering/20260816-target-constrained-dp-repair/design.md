# Design

## 方針

### 1. 相手境界をDP生成前に交差する

各distance sampleについて、昇格判定と同じ `resolve_receding_horizon_target_prediction` を用いて到達時刻の相手横位置と車体重複区間を求める。

- pass sideが正: `lower >= target_lateral + physical_separation`
- pass sideが負: `upper <= target_lateral - physical_separation`

物理離隔はhard boundsへ適用する。ロバスト離隔はpreferred boundsへ適用し、preferred intervalだけが消える場合はhard corridorを残す。

### 2. 生成経路と昇格検証の予測モデルを揃える

初回候補、直接Pass候補、longitudinal timing候補、rolling candidateの全てが上記制約後のcorridorをDPへ渡す。既存の昇格時target-bound validatorは独立した最終検証として維持する。

### 3. 不成立prefixを再利用しない

runtime target-bound hold中のrolling refreshでは、古いDP prefixを保存せず、測定中の横位置をanchorとして新候補へblendする。通常時の連続性維持では従来どおり古いprefixを保存する。

## 変更範囲

- `v2x_overtake_core.hpp/.cpp`: target-constrained corridor純粋関数
- `mpc_controller_cpp.cpp`: corridor生成統合とtarget-bound repair時の再anchor
- `test_v2x_overtake_core.cpp`: 左右制約、soft reserve低下、hard conflictの試験

## 非対象

- acados/MPC solverの崩壊対策
- Reverse/Recoveryパラメータ
- ROS interface、評価schema
