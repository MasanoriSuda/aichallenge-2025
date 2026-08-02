# Design

## 背景

`20260802-114402` では ShiftOut から Pass へ入れている一方、Pass 中に現在車体が
非重複でも 1 秒 horizon の予測重複が 0.25 秒続くと front cap が再適用され、直後に
longitudinal-only の SafetyBrake へ遷移していた。入口や横経路より、成立済み Pass の
速度所有権を予測だけで失うことが主要な失敗経路である。

## 方針

`v2x_overtake_committed_pass_attack_mode_enabled` を追加する。

1. `resolve_committed_pass_policy`
   - Pass、minimum-motion corridor、既解除、target 継続、現在 footprint 非重複、
     実行経路が物理的に成立、壁接触なしの場合、予測重複が confirm 済みでも解除を保持する。
   - 初回解除は従来どおり current + predicted footprint sweep の成立を要求する。
2. Behavior front danger
   - 同じ成立済み Pass では、固定 corridor と現在 footprint 非重複を根拠に、
     locked target に対する縦距離だけの SafetyBrake を抑制する。
   - 現在 footprint 重複、target 異常、別車両、inter-vehicle corridor は抑制しない。
   - 壁／solver は後段の OvertakeLine 実行監視を変更しない。
3. body-clear deadline
   - deadline 達成候補を優先する。
   - deadline 未達でも、動的 corridor と壁・横加速度 preflight が成立する候補は残す。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`
- `mpc_controller_cpp.cpp`
- `config/config.yaml`
- `test/test_v2x_overtake_core.cpp`

