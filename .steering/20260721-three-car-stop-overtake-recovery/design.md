# Design

## Supervisor SAFE_STOP

既存の`aggressive_sim_recovery_enabled`をdev3で有効化する。recoverable reasonは一定時間後に
`StopAndConfirm`へ戻し、候補、接触数、距離budgetを新しいepisodeとして再評価する。

`ClearanceWaitTimedOut`だけは例外とし、回廊が塞がれている間に再試行ループへ入れない。
既存の`safe_stop_clear_confirm_sec`だけclearが連続した後に`CheckClearance`へ戻す。

escape完了は、車体clearを必須としたまま、測定距離へ設定可能な許容差を加える。
初期値は0.10 mとし、`1.947 / 2.000 m`のような停止誤差による永久停止を防ぐ。

## 追い越し横分離

既存の`v2x_overtake_pass_front_overlap_lateral_clearance`はfront brake除外専用にし、
`v2x_overtake_line_min_target_separation`を追加してライン目標生成を分離する。
旧yamlとの互換用に、新しいキーがない場合だけ旧キーへfallbackする。

- line minimum target separation: 0.75 m
- front brake exclusion: 1.15 m
- ShiftOut maximum closing speed: 1.2 m/s
- early Pass maximum closing speed: 0.5 m/s
- entry front distance: 5.0 m

実測したヘアピンgap dropout対策である`active_gap_loss_hold_sec=1.0`は維持する。

## 衝突後Recovery入口

pure helperでsimulation-only overrideを判定する。設定有効、シミュレーション、collision hint、
front vehicle、前進意図、自車速度がstopped閾値以下、という条件をすべて要求する。
上書き後も通常の0.4秒無進捗確認、static/V2X rollout、gear/boost gateは残す。

V2X復帰回廊は、選択済みrolloutがある場合、相手速度にかかわらず時刻予測込みの
circle-obstacle clearanceを使う。rolloutがない場合だけ従来のmoving corridorへfallbackする。

## 影響範囲

- `stuck_recovery_core.*`: simulation-only deliberate stop overrideとSAFE_STOP遷移。
- `mpc_controller_cpp.cpp`: 設定読み込み、escape tolerance、V2X rollout適用。
- `config.yaml`: dev3向け暫定値。
- `v2x_overtake_core.*`: 独立した横分離設定の利用。
- `docs/spec/mpc-integration.md`: 現行仕様と2025/2026暫定の記録。

