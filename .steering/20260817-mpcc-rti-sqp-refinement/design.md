# Design

## 現状

現行の progress-contouring MPCC は、Mission / DP corridor が作った nominal
`[e_y, e_psi, s]` と `[v, kappa]` の周りで1回だけ線形化し、OSQPを1回解く。
そのため、QPが選んだ速度・操舵・横位置が nominal から離れても、その周期の
運動モデルは更新されない。

## 変更方針

制御周期ごとに次を行う。

1. shifted warm-start を使って従来QPを解く。
2. 前の線形化点と1回目の解を `alpha` で混合する。
3. 混合軌道の各stageで temporal Frenet modelを再線形化する。
4. 同じ目的関数、hard corridor、進捗trust regionで2回目のQPを解く。
5. 2回目が可行なら採用し、失敗なら1回目を採用する。

経路曲率 `k_ref` と制御入力曲率 `kappa` は別変数として扱う。これにより、
再線形化時にQPが選んだ操舵を経路形状と誤解しない。

## 影響範囲

- `mpcc_progress`: RTI-SQP設定、減衰更新、正しいFrenet線形化。
- `mpc_controller_cpp`: MpcProblemへ再線形化メタデータを保持し、OSQP solveを
  first-feasible + refinementへ局所リファクタする。
- `test_mpcc_progress`: 曲率分離と減衰更新のテスト。

Recovery、V2X契約、topic、launch、result JSONは変更しない。

## フォールバック

- 1回目失敗: 従来どおりsolver failure処理。
- 再線形化拒否: 1回目の解を採用。
- 2回目失敗: 1回目の解を採用し、次周期のwarm-startとして保持。
- MPCC準備拒否: 既存legacy MPCへフォールバック。
