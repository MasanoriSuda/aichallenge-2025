# V2X協調バックRecovery Design

作成日: 2026-07-18
状態: Completed

## 方針

新しいROS topicや中央指令は追加しない。各車両が既存`/v2x/vehicle_positions`と自車のV2X behaviorを
使い、同じルールで最後尾から順に動く分散方式とする。

## 状態系列

1. 前方の停止車によりSafetyBrake / Followとなった車両は、通常のdeliberate stopを維持する。
2. 前方車速度が閾値以下で停止が設定時間継続した場合だけ、coordinated stop候補へ昇格する。
3. Stuck Recoveryへ入るが、候補生成はReverse Straight / Left / Rightだけに限定する。
4. 後方車がcorridorにいる車両は`WAIT_FOR_CLEAR`で停止する。
5. 後方がclearな最後尾だけReverseへ入り、既存のbounded stepを実行する。
6. 最後尾の移動で次車の後方corridorがclearになれば、その車両がReverseへ進む。
7. solver failure車はwall証拠の有無によらず、必要な観測時間を満たした場合だけReverse候補を評価し、Forward fallbackは使わない。

## 実装境界

- `stuck_recovery_core`: coordinated deliberate stopを時間付きで許可するpure detector policy。
- `mpc_controller_cpp`: V2X behavior/front speedから候補を作り、episodeをlatchする。
- `evaluate_recovery_safety`: coordinated episodeとsolver failure episodeではforward fallbackを作らない。
- `config.yaml`: enable、確認時間、前方速度、方位誤差閾値。
- `test_stuck_recovery_core.cpp`: detector境界と既存deliberate stop回帰。

## 安全性

- coordinated stopはsolver healthyな後続車に適用する。
- solver failure車はwall証拠ありなら既存2.0秒、wall証拠なしなら前進要求・静止・pose / path無進捗を3.0秒確認して候補化する。
- rear static / V2X corridorがunknownまたはblockedなら駆動せず、clear確認を待つ。
- static swept footprintが接触を悪化させる場合は、reverse-onlyを破ってForwardへ逃がさずSafeStopする。
- 候補episode中にbehaviorが変化してもreverse-onlyをlatchし、Forwardへ変わらないようにする。
- front speedが非有限、target不明、LowSpeedAvoidance、race inactiveでは候補化しない。
- 実験値は2025 AWSIM限定で、2026公式値・実車値ではない。

## 非対象

- V2X message型へのRecovery状態追加。
- 後退完了後の全車一斉resume protocol。
- 追い越しパラメータやtrajectory CSVの変更。
- 実車でのReverse有効化。

## 実験で確認した境界

`output/20260718-011435`のD3ではreverse-only episodeへの遷移後、全Reverse候補が
`contact_worsened`となった。`maneuver_direction_unknown`でSafeStopし、Reverse gear requestは
publishしていない。安全gateを緩める変更ではなく、「計算不可なら安全な場合だけバックする」設計である。
