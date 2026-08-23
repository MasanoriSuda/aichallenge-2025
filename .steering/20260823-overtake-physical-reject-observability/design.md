# Design

## Root-cause boundary

直前Sliceでは Overtake canonical fresh shadow 1136 eligible 周期中、1130周期が完全な canonical candidate になり、6周期だけ物理証明で棄却された。solver、lateral row contract、primal、trajectory extraction は6周期とも通過している。

現行ログは1秒窓の件数では `physical=2/8` を示す一方、詳細は窓の最後の周期だけを表示する。このため最後が成功すると、6棄却の `PhysicalWallCertificateReason` と診断位置が失われる。

保持解を年齢だけで採用するのは不可。棄却が current pose contact / swept path violation の場合は、保持解も現在世界で再証明できなければならない。したがって本Sliceでは制御を変えず、棄却理由を確定する。

## Change

1. 物理証明理由別カウンタを共通構造へ集約する。
2. Track/Cruise の既存カウントを同構造へ移す（意味は不変）。
3. Overtake fresh shadow に同じ集計を追加する。
4. Overtakeでは最後に棄却された decision ID と完全な diagnostic を1秒集計へ残す。

## Non-goals

- retained Overtake authority の追加
- production authority 昇格
- solver/weight/margin tuning
- callback overrun対策
