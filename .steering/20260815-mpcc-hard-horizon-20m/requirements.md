# Requirements

## Purpose

Frenet DP が約30 m（ログ上の `plan_N=52`、経路解像度0.6 m）全域へ
実行時と同じ壁余裕を hard 制約として課し、遠方の狭窄だけで目前の追い越し候補を
棄却する問題を緩和する。

## Requirements

- 直近20 mは、実行時と同じ壁余裕を hard 制約として維持する。
- 20 mより先の既存約30 m先読みは残し、壁余裕を soft cost として評価する。
- 相手車両を横切らない左右別の corridor、現在車体・壁接触、runtime wall validation、
  rolling refresh は維持する。
- 2025由来のROS 2 topic/service/提出インターフェースを変更しない。
- `config.yaml` と `config_for_cloud.yaml` を同値に保つ。

## Non-goals

- `plan_N` を52点から20点へ直接短縮しない。
- 壁接触や現在車体overlapのhard guardを緩和しない。
- Recovery、V2X、評価基盤の契約を変更しない。
