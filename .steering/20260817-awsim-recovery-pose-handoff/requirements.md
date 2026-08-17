# Requirements

## 背景

AWSIM の衝突復帰が車体 pose / yaw を外部から補正した後、参加者側の Stuck Recovery が補正前の観測 anchor、走行距離、選択済み primitive、失敗履歴を引き継ぐ場合がある。特に車体が反転または大きく回転した場合、通常 rejoin として扱うと `rejoin_path_blocked` の再試行ループへ入り得る。

## 要件

- AWSIM 復帰待機中の pose / yaw 変化を外部 maneuver として検出する。
- AWSIM 復帰後に lateral / heading 誤差が rejoin 許容内でない場合、車体が clear でも Normal へ即復帰しない。
- AWSIM 復帰待機を抜ける際、現在 pose を基準に Recovery の局所履歴と軌道対応を再初期化する。
- 反転時は補正前の Reverse / Forward 方針を保持せず、現在姿勢と現在の壁・V2X 状況から maneuver direction を再評価する。
- 既存の ROS topic/service、評価 schema、通常追い越し条件は変更しない。
- ユーザーが変更中の `config/config.yaml` と `aichallenge/result-summary.json` は変更・stage しない。

## 完了条件

- aligned な AWSIM 復帰は従来どおり Normal に戻れる。
- heading error が許容外の AWSIM 復帰は `StopAndConfirm` へ進み、Normal に直接戻らない。
- pose handoff 後の観測・maneuver 履歴を現在 pose へ張り直すコードと単体テストがある。
- package test と `make autoware-build` が成功する。
