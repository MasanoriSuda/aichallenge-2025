# Design

## 1. 距離責務の分離

以下の二つを別変数として扱う。

- `actual_escape_distance_m`: odometryから積算した実移動距離
- `predicted_stop_distance_m`: 実移動距離 + calibrated stopping reserve

`recovery_escape_confirmed` は前者、`reverse_escape_brake_required` は後者を使う。
coreへ渡すper-maneuver/episode距離にも停止予測を加算しない。

これにより、予測停止点が4 mへ達したらReverse gear内でブレーキするが、実停止位置が
4 m未満なら同じReverseManeuverから再びcreepし、実距離が目標へ達するまで収束する。

## 2. Rolling stepwise Reverse

full 4/8 m rolloutが成立せず0.4 m候補だけが成立した場合、従来のstepwise safety
評価を維持しつつ、Supervisorへは連続Reverseとして渡す。これにより0.4 mごとの
Drive/Reverse gear切替を行わない。

各0.4 m境界でadapterのcommitted primitiveだけを解除し、次周期に現在姿勢から
Reverse候補を再評価する。episode/per-maneuver実距離は維持するため、4/8 mの目標と
最大距離guardは失われない。

rolling latch中はReverse候補だけを評価する。新しい0.4 m候補が不成立ならReverseのまま
停止して再評価し、static/V2X/collision-worseningを無視して突進はしない。

## 3. 設定

`rolling_stepwise_reverse_enabled` を追加する。`simulation_only: true`を要求し、
`config.yaml`と`config_for_cloud.yaml`で有効にする。

## 影響範囲

- `mpc_controller_cpp.cpp`: 距離分離、rolling latch/replan、設定・ログ
- `config.yaml`, `config_for_cloud.yaml`: rolling stepwise有効化
- `test_stuck_recovery_core.cpp`: 実距離到達前のbrake/resume契約テスト
