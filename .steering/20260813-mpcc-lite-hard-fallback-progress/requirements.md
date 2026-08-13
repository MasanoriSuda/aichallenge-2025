# Requirements

## 目的

20260813-125441 の実走で確認された MPCC-lite / receding-horizon の不整合を修正し、
不可能な横制約を旧来の固定 horizon へ黙ってフォールバックさせない。

## 対象

- receding-horizon の hard / soft failure の区別
- course progress に整列した warm start
- MPCC-lite shadow の last-feasible 再利用条件
- current-side hold の active Mission 評価
- shadow 評価時間の診断ログ

## 制約

- MPCC-lite shadow に制御 authority は与えない。
- ROS 2 topic / service / message 契約は変更しない。
- hard constraint を緩めない。
- 既存の Recovery と solver guard は維持する。

## 完了条件

- hard-bound conflict 時に baseline horizon が実行用として残らない。
- warm start が同じ配列 index ではなく、走行進捗に合わせて再サンプルされる。
- shadow last-feasible は同一 target / Mission generation / phase / side のみ再利用される。
- active Mission が current-side hold の第一候補になる。
- shadow 左右評価と解決処理の所要時間をログから確認できる。
- 対象 package の unit test と build が通る。
