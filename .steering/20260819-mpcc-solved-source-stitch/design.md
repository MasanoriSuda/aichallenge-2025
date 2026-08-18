# Design

## 方針

通常のrolling DP refreshで使用中の `stitch_frenet_dp_execution_refresh_path()` を、非同期MPCC solved sourceの昇格にも再利用する。

処理順は次とする。

1. 非同期MPCC解を現時刻のhorizonへalignする。
2. 現行DP参照またはfallback参照をnominalとしてtrust envelopeを適用する。
3. nominalをactive path、trust制限後のMPCC解をcandidate pathとしてstitchする。
4. 現行参照をpreserved prefix区間で保持し、blend endまでsmoothstep接続する。
5. 実測 `e_y`、`e_psi`、速度と横加速度reserveから到達可能な横位置へ投影する。
6. stitch後の全horizonをQP bound・車体footprint・壁距離で再検証する。
7. 全検証を通った場合だけ既存DP実行sourceとatomicに交換する。

## 使用する既存設定

- `v2x_overtake_mpcc_lite_same_side_max_lateral_adjustment`
- `v2x_overtake_mpcc_frenet_dp_refresh_preserved_prefix_distance`
- `v2x_overtake_mpcc_frenet_dp_refresh_blend_end_distance`
- `v2x_overtake_mpcc_frenet_dp_execution_envelope_enabled`
- `v2x_overtake_mpcc_frenet_dp_execution_lateral_accel_reserve_ratio`
- `v2x_overtake_guard_max_lateral_accel`

新規パラメータは追加しない。通常refreshとsolved handoffの接続規則を一致させ、調整箇所を増やさない。

## 局所リファクタリング

trust envelopeとrefresh stitchの合成をpure helperへ切り出す。controllerは物理再検証とstate交換を担当し、横経路接続の数値処理をcoreへ集約する。

## 非対象

- 壁クリアランス値の攻撃化・保守化
- MPCC QP目的関数の変更
- Recovery/Reverseの変更
- target選択、左右戦術の変更
