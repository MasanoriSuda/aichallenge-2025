# Tasklist

- [x] `output/20260808-235118/d1`の無制限Follow、Pass timeout、solver failureを照合する
- [x] requirements/designを記録する
- [x] Follow deliberate-stop判定をpure coreへ追加する
- [x] rearward-progress time graceをSafeSeparationへ追加する
- [x] config、起動ログ、仕様書を更新する
- [x] core unit testを追加する
- [x] package test/buildを実行する
- [x] `git diff --check`とinterface境界を確認する

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（925 tests、0 failures）
- 新規`FollowRequiresAnEffectiveLongitudinalRestriction`: 成功
- 新規`UsesDistanceAndFreshProgressAfterRearwardTimeLimit`: 成功
- `git diff --check`: 成功
- ROS 2 topic/service/message、Domain、評価結果schema、加減速度上限の変更なし
- `colcon test-result --verbose`は対象外の古い`build/joycon_contract_guard/package.xml`欠損を
  warning表示したが、対象packageの結果は0 errors / 0 failures
- 実装完了時点では`make dev2`による動的効果確認は未実施

## 動的確認項目

- `Follow / limit=inf / follow_cap=0`で`reject=deliberate_stop`が継続しない
- wall/contactありsolver fallbackでは約2秒、wallなしでは約3秒でRecovery候補になる
- target後方かつ進捗中のSafeSeparationで`rearward progress time grace`が出る
- 進捗が止まれば時間Abortまたはstuck recoveryへ移る
- `/control/command/control_cmd`が前方空間で加速度`+1.0 m/s^2`を維持する
- SafetyBrake、壁接触、Reverse、競争停止が増えない

## 動的検証結果（20260809-002226）

- `make dev2`で約523秒走行し、d1/d2ともrosbagを取得できた
- P1は6周、P2は3周。P1最速46.72秒、平均80.09秒
- P1は`Pass -> Return -> Idle`を2回完遂した一方、
  `SafeSeparation aborted: short horizon unsafe`が4回発生した
- `Follow / front=1 / limit=inf / follow_cap=0 / ego=0`で、
  `reject=deliberate_stop`ではなく0.27秒後に`Confirmed`となり、
  `FORWARD_MANEUVER`へ進んだ。非制限FollowがRecoveryを阻害する問題は改善を確認した
- 前進escapeは約0.208 mで完了したが、その直後にcoordinated stopが再確認され、
  Recoveryが再発火した。5回の`stuck_confirmed`と7回のReverse maneuverがあり、
  復帰後の再arm抑制不足が次の主課題
- `rearward_time_grace=enabled`は起動ログで確認したが、今回のSafeSeparation abortは
  すべてshort-horizon unsafeであり、rearward progress中のtime-limit事象は再現しなかった。
  この項目の効果は未判定
- したがって本runは部分改善だが、SafetyBrake／short-horizon abortとRecovery再発火により
  54.82～109.53秒の異常周が残り、提出安定版としては不合格
