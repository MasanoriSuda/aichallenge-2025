# Pass live-corridor phase hold design

## 原因

live corridorのholdは`last_valid_execution_corridor_sec`だけを基準にする。plannerが
一時的にinactiveになる周期はログ上`corridor=valid`と表示されても、この時刻を更新しない。
そのため、`ShiftOut`で始まった不成立区間が`Pass`へ遷移しても同じ期限で評価され、
Pass用の明示lineを継続して検証する時間が残らない。

## 方針

live execution corridorのhold基準時刻を次のように解決する。

```text
reference = last_valid_execution_corridor_sec

if Pass && current body footprints are separated:
  reference = max(reference, Pass phase start time)
```

これにより、ShiftOut中のholdは従来どおり自己延長せず、Passへ入った時だけフェーズ固有の
最大2秒が与えられる。Pass中にlive corridorが再び成立すれば、実際の最終有効時刻が
基準を更新する。

## 安全境界

この変更が緩和するのは、gap plannerのlive corridor不成立だけである。現在車体が非重複で
あることを要求し、target continuityとEmergencyBrake等は既存のhold resolverで拒否する。
OvertakeLine側のactual/static wall、target intrusion、予測overlap、solver、odometryの
各guardは変更しない。

## 変更箇所

- `v2x_overtake_core.hpp/.cpp`
  - Passフェーズ用hold基準時刻を解決する純粋関数を追加する。
- `mpc_controller_cpp.cpp`
  - live execution corridor holdへ解決済み基準時刻を渡す。
  - debug logへphase-scoped基準の利用有無を出す。
- `test_v2x_overtake_core.cpp`
  - 非Pass、車体重複、Pass開始、Pass後の再成立を単体テストする。
- `docs/spec/mpc-integration.md`
  - フェーズ境界のhold規約を追記する。

