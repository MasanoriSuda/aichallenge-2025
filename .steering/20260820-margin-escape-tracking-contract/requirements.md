# Requirements

## 背景

`output/20260820-131621` では、dynamic obstacle escape の static wall
preflight が `margin-escape` を採用した約 10 ms 後に、同じ candidate の追従
QP が `maximum iterations reached` で失敗した。

現行実装は、実寸 footprint が安全で、短距離内に通常の壁余裕へ戻る経路を入口で
許可できる。一方、その「初期だけ通常余裕を継承する」という条件は追従QPの
stage境界へ渡っておらず、入口と追従で異なる可行集合を使っている。

## 要求

- 実寸 footprint の壁非接触は全stageでhard constraintとして維持する。
- `margin-escape` 採用時だけ、現在横位置が通常の壁境界外なら、復元距離まで
  壁由来の境界を段階的に通常値へ戻す。
- 車両によって狭められた境界は緩和しない。
- 解けた追従軌道も実寸footprintで再検証し、不安全な解は制御へ渡さない。
- planning traceだけで、境界継承の採否、緩和stage数、復元距離、最大緩和量、
  拒否理由を特定できる。
- 通常clear candidateと既存Overtake Missionの境界を変更しない。

## 制約

- ROS 2 topic、message、launch、評価インターフェースは変更しない。
- clearance設定値を緩めない。
- `output/`、result生成物、ユーザー所有ファイルを編集・コミットしない。
