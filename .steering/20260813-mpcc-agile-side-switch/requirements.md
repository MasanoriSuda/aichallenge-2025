# Requirements

## 目的

MPCC-liteが走行中に左右候補を比較していても、旧来のside-replanゲートに阻まれて反対側の完全Missionへ切り替わらない問題を解消する。

## 要件

- ShiftOut開始後も、左右の完全Missionと現在側維持を継続評価する。
- targetが十分前方にあり、現在車体が非重複である間は、MPCC-liteの優位な反対側候補へ切替可能とする。
- 反対側候補は壁・車体・横加速度・rear-clear・Returnまで成立した完全Missionに限定する。
- 一瞬の候補変動では切り替えず、短い安定確認と最小スコア差を要求する。
- side-by-side/no-return到達後は反対側への全幅切替を禁止し、現在側で前進完遂する。
- 1回のside切替だけでMission全体を永久固定せず、no-return前に限り複数回の再選択を許可する。
- hard wall fault、実車体重複、target不連続、EmergencyBrakeを緩和しない。

## 変更範囲

- `multi_purpose_mpc_ros`のMPCC-lite authority、cross-side Mission置換、設定、単体テスト。
- ROS topic/service/message契約は変更しない。

