# Results

## 実施条件

- 日時: 2026-07-19
- 出力: `output/20260719-233608`
- 起動: `make dev3`
- 設定:
  - `v2x_overtake_min_gap_width: 0.2`
  - `v2x_overtake_guard_min_gap_width: 0.2`
  - `v2x_overtake_line_min_wall_clearance: 0.1`
- `make autoware-build`: 25 packages成功
- 終了後に`make down`を実行し、対象コンテナが残っていないことを確認した。

## 観測結果

- V2X debugのgap拒否ログが`req=0.2`となり、plannerとguardへ変更値が読み込まれた。
- D1は3回`ShiftOut -> Pass`へ入ったが、いずれも`locked target no longer executable`から
  Recoveryへ移行した。Returnへの移行はなかった。
- D2は1回`ShiftOut -> Pass`へ入ったが、その周期でRecoveryへ移行した。このほかの
  ShiftOutはhard curveまたは同じlocked-target継続判定で終了した。Returnへの移行はなかった。
- D3ではOvertakeLineの開始を観測しなかった。
- 旧0.8 m未満の残余幅について、0.290〜0.673 m、連続1〜4点の候補を観測した。
  0.2 m化により一部区間は幅条件を満たすが、選択側の経路全体はpass-side feasibility、
  hard curve、completion条件などで不成立のままだった。
- 実際に選ばれた追い越し側の`side_clear`はおおむね2.56〜3.97 mであり、今回の走行では
  0.2 m級の狭い回廊を実際に通過したケースは得られなかった。
- `wall_limited=1`は全domainで0件だった。
- 走行中の新規OSQP failureはなかった。D2に31件のOSQP errorログがあるが、すべて
  発走前の速度0 m/s時点で、failure counter 1〜290の同一startup episodeだった。
  D1/D3は0件だった。
- 明示的なcollisionログ、Reverse/SafeStopを伴うstuck recoveryは観測しなかった。

## 判定

今回の3値による接触、停止、走行中solver悪化は観測されなかったため、dev3シミュレーション
A/B値として維持する。一方、追い越し完了率の改善と狭い車両・壁間を通る際の影響は未確認である。
今回の追い越し失敗はgap幅よりもhard curve/completionとlocked target継続判定が支配的だった。
次回は自然に狭い側だけが選択される場面で、横偏差、壁clearance、接触有無を再評価する。
