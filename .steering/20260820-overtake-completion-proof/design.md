# Design

## 原因モデル

現行は次の不整合を許している。

1. entry side clearanceをfuture interaction reserveより先に比較する。
2. 相手横位置を現在の横速度だけで予測し、0.6秒で横速度を減衰させる。
3. complete Missionが得られない場合、local ShiftOut/body-clearだけを証明した
   progressive候補を近距離でも採用できる。
4. squeeze検知時にはno-return近傍で、壁と相手の間に新しい横目標が残らない。

## 変更

### 1. Completion ordering

`select_overtake_mission_candidate()`で、両候補のPass相手余裕が計測済みなら、
入口幅より先にinteraction reserveを比較する。入口幅は将来余裕が同等または
未計測の場合のtie-breakへ下げる。

### 2. Lateral uncertainty tube

kinematic rolloutへ次を追加する。

- 横不確実性成長率 `[m/s]`
- 最大横不確実性 `[m]`

時刻`t`の離隔余裕から`min(rate * t, max)`を差し引く。設定0では従来挙動を
完全に維持する。初期設定は0.10 m/s、上限0.25 mとする。

### 3. Progressive completion-proof gate

rear-clear未証明の新規entryだけを対象とし、次の両方を要求する。

- target front distance >= 8.0 m
- `(front - no_return_distance) / positive_closing_speed >= 1.0 s`

complete Mission、active Missionの同側receding prefix、停止相手に対してclosingが
ほぼ0の遠距離候補はこの新規entry gateで一律停止させない。

### 4. Diagnostics

既存の候補理由へ次を追加する。

- `completion_proof=complete|local-only`
- `lateral_uncertainty=rate/max`
- `progressive_gate=reason`
- `time_to_no_return`

新規entryがgateで拒否されたときだけ、throttle付きWARNを1秒周期以下で出す。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: 純粋判定、rollout、候補順位
- `mpc_controller_cpp.cpp`: 設定読込、判定接続、診断
- `config.yaml`: sim向け既定値
- `test_v2x_overtake_core.cpp`: 回帰試験
