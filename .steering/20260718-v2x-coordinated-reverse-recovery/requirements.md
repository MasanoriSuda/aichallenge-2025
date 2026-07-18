# V2X協調バックRecovery Requirements

作成日: 2026-07-18
状態: Completed（安全性確認済み、全車停止の解消は未達）

## 背景

`output/20260718-003725`ではD3がWP272付近で`e_psi=-1.808 rad`のまま連続solver failureへ
入り、後方にD2がいるため既存RecoveryはForwardを選択した。Forwardは目標`0.300 m`に対して
`0.123 m`で時間上限となりSafeStopし、D2とD1も前方車へのSafetyBrakeで停止した。

単独車のReverse Recoveryは既に存在するが、後続車がいる多車両列では先頭車だけをバックさせると
追突する。最後尾から順に後方空間を作り、前方のsolver failure車が安全にバックできる分散協調が
必要である。

## 要求

1. solver failureを伴わない通常のSafetyBrake / Follow停止は、短時間では従来どおり意図的停止として扱う。
2. 停止した前方車が低速で、一定時間、自車速度・pose・コース進捗が止まった場合だけ協調バック候補にする。
3. LowSpeedAvoidance、moving front、V2X欠落、古いodometry、非単調時刻では協調バックしない。
4. 協調バック候補はForwardへ切り替えず、静的swept footprintとfresh/completeなV2X後方corridorがclearになるまで停止する。
5. 大きな方位誤差を伴うsolver failure車もreverse-onlyとし、後続車が空間を作るまで停止する。
6. wall証拠がない連続solver failureも、前進要求・静止・pose / path無進捗がより長い時間継続した場合だけRecovery候補にする。
7. solver failure由来のRecoveryはForward fallbackへ切り替えず、Reverse候補だけを評価する。
8. Reverse gear reportを確認するまで駆動せず、既存の距離・速度・時間・停止距離予約を維持する。
9. static rolloutが接触悪化、unknown、map外となる場合はギアを要求せずSafeStopする。
10. 協調Recoveryの候補化、待機、選択、駆動をログで区別できる。
11. ROS topic、service、message、Domain、評価JSON契約は変更しない。
12. simulation-only制約を維持し、実車では有効化しない。

## 2025 AWSIM向け暫定値

- 協調停止確認: 3.0秒
- 停止前方車速度上限: 0.20 m/s
- wall証拠なしsolver failure確認: 3.0秒
- solver reverse-only方位誤差: 1.0 rad
- 後退動作は既存の0.40 m step、最大0.8 m/s、最大3.0 mを流用する。

## Definition of Done

- pure detectorで通常deliberate stopが除外され続けることを確認する。
- 協調停止だけが設定時間後にConfirmedとなることを確認する。
- 無効設定、moving、観測gap、solver failureとの境界をunit testする。
- `make autoware-build`と対象unit testが成功する。
- `make dev3`でD1〜D3の候補化順、rear corridor、gear、移動距離、接触、最終進捗を記録する。
- 接触または後方不完全情報でReverseCreepを出した場合はFailとする。
- 全車停止が解消しない場合も、協調バックが発火しなかった理由をログ証拠で特定する。

## 受け入れ結果

- 64件の対象unit testと25 package buildは成功した。
- `output/20260718-011435`でD1は2.059 mの協調後退、D2は後方車検出まで0.148 mの協調後退を実行した。
- 同runのD3は連続solver failureからreverse-only Recoveryへ入ったが、0.05 m先からstatic contactを悪化させるため、逆ギアを要求せずSafeStopした。
- 安全な候補がない場合の強制後退は行わない。したがって全車停止の解消は未達で、次段はstatic contactから離脱できる候補生成を別ステアリングで扱う。
