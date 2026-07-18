# Requirements

## 背景

`output/20260718-182347`では前回の停止車V2X rollout判定は機能し、D2は2回にわたり
2 m以上の後退と`rejoin_complete`へ到達した。一方、復帰途中の一時的な条件変化が
解除不能なSafeStopへ直結し、最終的に全車が停止した。

- D1: ForwardManeuver中に移動中D2が現れ、`forward_hazard_appeared`でSafeStop。
- D3: LowSpeedRejoinが5秒で整列せず、`rejoin_timed_out`でSafeStop。
- D2: 3回目の復帰がmap clearかつLeft wallで非stepwiseになり、0.544 mで時間上限へ達して
  `escape_not_confirmed`でSafeStop。

初回修正後の`output/20260718-184701`ではD2がstepwise退避から`rejoin_complete`へ復帰したが、
D3は後壁検出中のsolver fallbackにもかかわらずevidence-free設定だけで`reverse_only=1`となり、
前進のみ安全な後壁条件と衝突して`maneuver_direction_unknown`でSafeStopした。

## 必須要件

1. ForwardManeuver中のclearance喪失は即時停止するが、停止後に全安全条件を再評価できる。
2. current footprintがclearでもLeft / Right / Mixed wallなら0.4 m stepwise復帰を使用する。
3. 非stepwise後退が時間上限へ達した場合、残りattemptがあればDriveへ戻して再評価する。
4. LowSpeedRejoin timeoutは設定で有効化し、距離・step・attempt上限内だけ再離脱を許可する。
5. rejoin retryでは以前のescape距離を完了条件へ流用せず、新しい離脱距離を0から測る。
6. static map、V2X、gear report、collision worseningのfail-closed判定を緩めない。
7. 実車は既存のsimulation-only gate外へ広げない。
8. solver fallbackは、壁なし時または姿勢誤差上限以上だけreverse-onlyとする。壁ありかつ姿勢誤差
   上限未満では、壁方向が選んだ候補を既存のstatic / V2X gateで検証する。
9. Mixed / side contactの短距離escapeは前後両方向を同じcontact改善条件で比較し、改善量最大の
   候補だけを許可する。改善候補がなければSafeStopを維持する。
10. solver fallbackを起点に開始したepisodeは、独立低速rejoinが完了するまで資格を保持する。
    通常のrecovery中に新規発生したsolver failureのHoldStop / timeoutは維持する。

## 完了条件

- 新しい遷移のunit testが通る。
- `multi_purpose_mpc_ros`がbuildできる。
- `make dev3`で各車の最終状態と、再評価が実行された証拠を確認する。
- 完走しない場合も、新しいblockerと残課題をrun単位で記録する。
