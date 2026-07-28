# LowSpeed静的壁preflight整合化 設計

## 原因

停止車local path plannerはwaypointの`lb/ub`へscalar wall marginを加えた
Frenet区間でsideを選ぶ。一方、direct controlのlive guardはoccupancy grid上で
向きを持つ実車矩形をrasterizeする。カーブや車体前後長を含む物理条件が開始判定に
入っていないため、選択直後の実走poseでwall margin違反になり得る。

またdirect controllerはlocal pathの緩やかな`target_ey` rampではなく、選択した
最終`pass_target_ey`を即時に追う。このためpreflightはMPC用rampだけでなく、
現在raw poseから最終target側へ移る掃引を評価する必要がある。

## 変更方針

### 1. 汎用静的path掃引

`recovery_footprint`へ、複数の`Pose2D`を結ぶpathを実車矩形で検査するpure APIを
追加する。各segmentは、

`並進距離 + footprint角半径 * yaw変化量`

がmap resolution以下になるよう細分化し、途中poseも全て
`sample_footprint()`で確認する。invalid、map外、unknown、occupiedはfail closedとする。

### 2. LowSpeed preflight

MPC controller内で次のpose列を作る。

1. raw odometryの現在pose
2. horizon各waypointを`pass_target_ey`だけ横移動したpose

footprintのleft/right extentへ既存`v2x_wall_clearance_margin`を加え、
上記pathを掃引する。これはdirect controllerが最終targetを早期に追う場合を
含む保守的な実行可能性確認である。

### 3. side再選択

停止車local path plannerへ内部用のforced side引数を追加する。

- 最初のcandidateがpreflight clear: そのsideを使用
- 最初がblockedかつ`v2x_low_speed_pass_side=auto`: 反対sideを再計画・再検証
- 両方blockedまたはfixed side blocked: candidateをinfeasibleとして返す

side lockとplanner state更新は、静的preflightを通ったcandidateだけに行う。
反対sideの再選択はdirect control開始前だけとし、開始後はtargetを反転しない。
実行中も同じwrapperで再評価し、地図・経路変化時はlive corridor unavailableとして
停止する。既存raw-pose wall guardは削除しない。

## ログ

preflight棄却時はside、理由、checked pose数を出す。両side不成立が続く場合の
周期ログは抑制し、状態変化または一定間隔だけ記録する。

## インターフェース影響

- ROS interface: 変更なし
- parameter schema/default: 変更なし
- 変更範囲: `aichallenge_submit/multi_purpose_mpc_ros`と正本仕様文書のみ
