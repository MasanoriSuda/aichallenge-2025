# Design

## 1. 拡張MPCC専用追従重み

旧MPCの `Q/QN` に単一係数を掛ける方式を廃止し、以下を独立設定する。

- stage lateral: 500
- stage heading: 5000
- terminal lateral: 1500
- terminal heading: 5000

これにより、数値安定化のためのスケーリングと追従性能を分離する。旧 `progress_contouring_extended_tracking_weight_scale` は外部設定との互換用fallbackとしてloaderだけで受理する。

## 2. mode handoff

拡張MPCCと3-state MPCCが切り替わったとき、直前の速度指令から新しい解へ0.15秒で補間する。

補間値は毎周期の `progress_input_lower/upper` 内へclipする。したがって、前方危険などで上限が下がった場合は補間せず即時に新しい上限を適用する。操舵は既存のsteer-rate制限を維持する。

## 3. 維持する処理

- thetaのローカル座標化
- extended warm-startのprogress origin再基準化
- 0.75秒のfailure circuit breaker
- 物理壁検証、front-risk cap、solver failure fallback

## 影響範囲

- `mpcc_progress.hpp/.cpp`: 専用重み、mode handoff helper
- `mpc_controller_cpp.cpp`: QP cost、設定load、mode handoff適用、起動ログ
- `config.yaml`: 新しい専用重み
- `test_mpcc_progress.cpp`: hard boundを守るhandoffの単体試験
