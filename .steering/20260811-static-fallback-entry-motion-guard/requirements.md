# Requirements

## 目的

`output/20260811-171932` で発生した、動的 corridor 未観測のまま大きな横移動を
開始し、壁際の solver failure と長時間Recoveryへ至る事象を防ぐ。

## 観測事実

- 失敗Missionは `corridor_source=static_fallback`、`dynamic_valid=0` だった。
- 自車横位置 `-0.83 m` から目標 `2.25 m` へ、`3.08 m` のShiftOutを採用した。
- 開始約1.1秒後にlive wall preflightがcollisionを報告し、その後target喪失、
  solver failure、Reverseを含む約12.5秒の復帰へ移った。
- 同じ走行の正常完遂Missionもstatic fallbackを使用したが、横移動は
  `0.68 m` と `1.21 m` だった。

## 必須要件

1. dynamic corridor未観測時のstatic fallbackは全面禁止しない。
2. 新規Mission entryに限り、static fallbackで許可する横移動量を設定値で制限する。
3. 上限超過候補だけを棄却し、同じ側のより小さい横目標と反対側候補の探索を続ける。
4. dynamic corridorで検証された候補、direct Pass、active Missionの再計画は対象外とする。
5. 候補が残らない場合はFollowを継続し、ShiftOut開始後のRecoveryにしない。
6. ROS 2 topic/service、launch、評価インターフェースは変更しない。
7. ユーザーの `aichallenge/result-summary.json` は編集しない。

## 完了条件

- pure core testでstatic fallbackの上限内／上限超過、dynamic候補、無効入力を固定する。
- dev/cloud設定を同じ値にする。
- candidate rejectionログに専用棄却数を出す。
- 対象packageのbuild/testが成功する。
