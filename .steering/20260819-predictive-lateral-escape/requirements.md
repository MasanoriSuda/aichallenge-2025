# Requirements

## Purpose

Pass 中に将来の壁接近を検出した際、前進を諦めて FollowPrepare / Recovery
へ落ちる前に、同じ pass side のまま実行可能な横逃げ prefix を生成する。

## Observed problem

- 最新走行では `Pass -> Return` が 0 回で、Pass の終了はすべて
  `FollowPrepare` だった。
- 一部 episode は前後関係をほぼ反転できていたが、runtime wall escape が
  `no wall-feasible physical-clearance centerward goal` で不成立になった。
- 現行実装は対車間をロバスト推奨値から実寸へ縮退できる一方、壁側の横目標
  interval はロバスト planning clearance に固定されている。

## Requirements

1. ロバストな対車間・壁余裕を第一候補にする。
2. 現在車体が非重複で hard wall fault がない場合だけ、実寸対車間および
   hard wall clearance を用いた同側横逃げを第二候補にする。
3. 実寸余裕候補にも既存の壁 footprint、横加速度、DP prefix preflight を
   すべて適用する。
4. pass side、target ID、Mission generation、front-cap release 状態を維持し、
   反対側への急な全幅横断は追加しない。
5. 壁 warning を早め、成立する横逃げの探索時間を確保する。
6. hard wall contact、車体重複、target discontinuity、solver 異常を緩和しない。
7. ROS 2 topic / service / message と評価インターフェースを変更しない。

## Out of scope

- 全車両を同時に扱う汎用 dynamic-obstacle MPCC
- no-return 後の反対側 branch への切替
- Recovery / Reverse の再設計
- solver failure の救済変更
