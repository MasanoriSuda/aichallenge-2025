# Requirements

## 目的

`20260819-164659`で確認した、Pass中のSafeSeparationがsoft abort後も毎周期
`Pass`を保持し、接触ペナルティ相当の低速状態から抜けられない不具合を解消する。

## 要件

- SafeSeparationのsoft abort後に許可する同側再計画待ちは、一回限りの有限リースにする。
- fresh same-side Missionまたはlast-feasible maneuverを採用できなければ、リース終了後に既存の確定handoffへ一度だけ移る。
- Pass中に十分な速度から低速へ落ち、前進進捗も一定時間止まった場合は、通常のPass継続として扱わない。
- receding-horizon MPCCがrobust/configured clearanceを縮退するときも、対象車体の実寸境界へ10 cmの実行余裕を残す。
- runtime wall preview/action分離とhard wall guardは変更しない。
- ROS 2 topic/service/message契約を変更しない。

## 非対象

- AWSIM固有の`1.389 m/s`を直接判定する処理
- solver/modelの変更
- Recovery/Reverse全体の再設計
