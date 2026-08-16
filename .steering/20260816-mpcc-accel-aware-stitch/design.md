# Design

## 背景

`20260816-094404` と `20260816-094823` では、DP execution envelopeの計画上限が5.4 m/s²であることを起動ログで確認できた。一方、rolling candidateのpendingは計6件すべて `ay=1` で、Pass完遂は0件だった。

raw DP sampleは実測 `e_y` と `e_psi` 由来の横速度から到達可能区間を作る。しかし後段のrolling stitchは、active pathを使わないmeasured rebase時に約1 mのprefixを現在 `e_y` へ固定する。横速度が非ゼロなら、これは短時間で横速度を打ち消す参照となり、実行前validatorが6.0 m/s²超過として補正する。atomic promotionは補正されたhorizonを採用しないため、fresh candidateがpendingとなる。

## 方針

### acceleration-aware stitch

stitch requestへ次を渡す。

- 実測横速度
- 現在速度
- DP計画用最大横加速度
- 到達可能性制約の有効状態

各path距離 `s` の到達時間と到達区間はraw DPと同じ式を使う。

```text
t = max(0.15, s / current_speed)
zero_ay_ey = current_ey + measured_lateral_velocity * t
reachable = zero_ay_ey +/- 0.5 * planning_max_ay * t^2
```

active pathがない場合、prefix anchorは一定の `current_ey` ではなく `zero_ay_ey` とする。active pathがある場合は従来どおり未消費prefixをanchorにする。その後、anchorとfresh candidateのsmoothstep blend結果を到達区間へ射影する。

これにより以下を両立する。

- healthy refreshではlast-feasible prefixとの連続性を維持する。
- measured rebaseでは実測横運動を急停止させない。
- 以前の実測状態から作られたactive prefixが現在状態から到達不能なら安全側へ射影する。
- 実行validatorの6.0 m/s²上限に対して5.4 m/s²の計画余裕を残す。

### 局所リファクタ

controller内の2つのstitch呼び出しをrequest生成lambdaへ集約する。初回refreshとmeasured rebase retryが同じ実測状態・計画上限を使うようにし、設定漏れを防ぐ。

### 診断

既存のrate-limit済みpending/refreshログへ次を追加する。

- stitch到達可能性制約が有効だったか
- stitch結果が到達区間へ射影されたか
- stitchで観測した最大必要横加速度

周期ログは追加しない。

## 互換性

到達可能性制約が無効なら従来stitchを維持する。参加者ROS I/O、評価FSM、Domain、launch entry、提出物構造、result JSONは変更しない。
