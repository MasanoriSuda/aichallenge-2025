# Requirements

## 目的

追い越しの左右・横位置・closing speed を、従来の「rear-clear と Return まで一括成立した Mission」だけで決めず、短い実行 horizon を反復評価する MPCC-lite の結果から選べるようにする。

## 背景

20260813-181149 の MPCC-lite shadow は 133 回評価されたが、best=none が 125 回だった。左右枝の大半が `planning_unavailable` で、既存の完全 Mission gate が shadow 候補にも適用されていた。これでは receding horizon の候補が optimizer へ到達しない。

## 必須要件

- body-clear までの局所 path、壁、target、横加速度を検証した prefix を MPCC 候補として扱う。
- rear-clear/Return が horizon 外でも、局所 prefix が成立していれば候補探索を継続する。
- MPCC winner は新規 ShiftOut の side / goal / closing speed に実制御 authority を持つ。
- active Mission の no-return 後は反対側へ切り替えない。
- hard wall fault、EmergencyBrake、solver recovery では MPCC authority を与えない。
- 同じ target の一時的な optimizer miss では、短時間だけ last-feasible prefix を保持する。
- target、Mission generation、phase、side が変わった last-feasible を再利用しない。
- ROS 2 topic/service、評価 JSON、Domain 契約を変更しない。
- ユーザー変更中の Domain 2 速度 15 km/h と `aichallenge/result-summary.json` を保持する。

## 非対象

- Recovery / Reverse の全面最適化
- ROS interface の変更
- 既存 hard wall/contact guard の撤廃
- 追従 MPC 全体の置換

## Definition of Done

- 局所 prefix の admission と authority 判定を pure function でテストする。
- MPCC-lite が選んだ局所 prefix を新規追い越し entry へ渡せる。
- active Mission では current-side/Return を保持し、無検証の cross-side replacement を行わない。
- package test と `make autoware-build` が成功する。
