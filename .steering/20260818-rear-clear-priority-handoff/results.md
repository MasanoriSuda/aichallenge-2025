# Results

## 実装結果

- Pass / SideBySideCommitted / forward-completion latch済みでtargetが後方の場合、
  target/physicalの将来軌道再検証失敗を即FollowPrepareへ変換しない判定を追加した。
- 現在body分離とcorridor、target continuityを必須とし、predicted sweep分離または
  SafeSeparationの新鮮な前進実績をcompletion evidenceとして使用する。
- 現在横位置を保つhorizonをwall footprintと横加速度で再検証し、成立した場合だけ
  rear-clear確認までPass authorityとprogress-contouringを維持する。
- hold検証距離は固定3mではなく、rear-clear残距離＋確認時間の走行距離＋path解像度へ限定した。
- rear-clear確認後は既存のvalidated Returnへ移行する。Return条件や2.0m閾値は変更していない。
- wall contact/margin/sample異常、EmergencyBrake、solver Recovery、forbidden waypointは
  従来どおりholdを禁止する。
- rear-clear前後で重複していたcurrent-side horizon評価をcontroller内lambdaへ集約した。
- 発火確認用に`OvertakeLine imminent rear-clear Pass hold`ログを追加した。

## 静的検証

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 28 / 28成功
- 対象package集計: 1252 tests、0 errors、0 failures、0 skipped
- 新規rear-clear境界test: 3 / 3成功
- `git diff --check`: 成功

## 動的確認項目

比較元は`output/20260818-131429`。

- `OvertakeLine imminent rear-clear Pass hold`がtarget_s<0で発生すること。
- 同じ箇所で`Pass -> FollowPrepare, reason=...physical revalidation`が減ること。
- hold後に`Pass -> Return -> Idle`へ直接完遂すること。
- progress-contouringの不要なOFF/ONとwarm-start resetが減ること。
- wall contact/margin、EmergencyBrake、solver faultではholdログが出ないこと。
- Pass中の最低速度、接触、wall Recoveryが悪化しないこと。

動的効果確認は次回`make dev2`試走で行う。
