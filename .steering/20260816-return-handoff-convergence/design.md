# Design

## 現状の問題

現行の完了条件は概念的に次である。

```text
phase_hold_elapsed
AND return_corridor_clear
AND (return_distance_reached OR abs(e_y) < 0.15)
```

距離条件がORであるため、カーブ中に横位置または姿勢が通常経路へ収束していなくても、Return参照を破棄して通常MPCへ切り替えられる。

## 方針

### 収束ベースhandoff

pure policyへ以下を渡す。

- Return active
- phase hold完了
- corridor clear
- solver ready
- 現在時刻と前回の収束開始時刻
- `e_y`と`e_psi`
- 横位置・姿勢の許容値
- 連続確認時間

瞬時収束条件は次とする。

```text
abs(e_y)   <= 0.20 m
abs(e_psi) <= 0.12 rad
corridor clear
solver ready
```

これが0.15秒連続した場合だけhandoffを許可する。観測不正、corridor block、solver recovery、閾値逸脱では確認時計をリセットする。
直前周期がsolver fallbackだった場合もsolver readyとは扱わず、正常解が復帰してから確認をやり直す。

### 距離の役割

`mission_return_distance`はReturn軌道のblend距離として維持するが、完了判定には使わない。距離を超えるとlegacy/return preflight参照の終端横位置を保持し、車両が実際に通常経路へ揃うまでReturn ownerを継続する。

### 診断

- 初めて距離を超えて未収束だった時に`Return handoff deferred`を1回出す。
- 収束完了時に`Return handoff confirmed`を1回出す。
- 両ログに`e_y`、`e_psi`、安定時間、走行距離を含める。

## 互換性

参加者ROS I/O、評価FSM、Domain、launch entry、提出物構造、result JSONは変更しない。新規設定はyaml省略時にも同じ安全側既定値を使う。
