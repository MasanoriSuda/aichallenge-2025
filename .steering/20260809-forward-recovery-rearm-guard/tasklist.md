# Tasklist

- [x] `output/20260809-002226/d1`のForward Rejoin再発火を時系列照合する
- [x] requirements/designを記録する
- [x] pure rearm guardとdetector rejectを実装する
- [x] adapterのForward Rejoin anchor、arm／releaseログを実装する
- [x] configと永続仕様を更新する
- [x] unit testを追加する
- [x] `make autoware-build`とpackage testを実行する
- [x] `git diff --check`とinterface境界を確認する

## 動的確認項目

- Forward `RejoinComplete`直後にguardが1回だけarmされる
- 3.0秒または前進3.0 mまで`reject=recovery_rearm_guard`となる
- ガード中もSafetyBrake／Follow速度上限は維持される
- 新しいcollision、wall evidence、solver fallbackで即解除される
- `stuck_confirmed -> Reverse`の短周期反復が減る
- Pass／Return成功率と異常ラップが悪化しない
