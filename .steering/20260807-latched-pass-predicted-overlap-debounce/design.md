# Design

## 方針

既存の `v2x_overtake_pass_predicted_overlap_confirm_sec` と
`update_predicted_footprint_overlap_confirmation()` を再利用する。

`CommittedPassBodyGeometryResolution` の予測重複確認対象に、既に認可済みの
forward completion を追加する。side-by-side であっても latch 済みなら確認時計を進める。

`CommittedPassForwardCompletionRequest` には予測重複の確認結果を渡し、次の条件で保持する。

```text
予測 sweep 非重複
OR
(forward completion latch 済み AND 予測重複が未確認)
```

初回認可では latch が存在しないため、この例外は利用できない。

## 即時中止を維持する条件

- 現在車体の確定重複
- 予測自体が無効
- 壁接触または壁サンプル欠損
- target discontinuity / position jump
- pass-side intrusion
- EmergencyBrake
- solver recovery

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: pure policy の入力・判定
- `mpc_controller_cpp.cpp`: 既存確認時計を forward completion latch へ接続
- `test_v2x_overtake_core.cpp`: 境界テスト

