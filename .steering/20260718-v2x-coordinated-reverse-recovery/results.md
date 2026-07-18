# V2X協調バックRecovery Results

実施日: 2026-07-18
結論: 安全gateはPass、全車停止解消はFail

## 実装・静的検証

- solver fallback中も、wall証拠なしで前進要求・静止・pose / path無進捗が3.0秒継続した場合だけRecovery候補にする経路を追加した。
- solver failureと協調停止由来のepisodeをReverse-onlyへ固定し、Forward fallbackを禁止した。
- 後方V2X、static swept footprint、gear report、距離・時間・速度上限は既存gateを維持した。
- ROS topic、service、message、Domain、評価JSON契約は変更していない。
- 対象unit testは8 suite、64 testsが成功した。
- `make autoware-build`は25 packageすべて成功した。

## dev3実験

### `output/20260718-005948`

D1がWP220付近でwall証拠なしの連続solver failureへ入った。旧条件では
`solver_fallback_missing_wall_evidence`となり、Recoveryへ入れなかった。この結果から
wall証拠なしsolver failureの3.0秒確認経路を追加した。

### `output/20260718-010936`

- D1は協調Reverseを8 step実行し、累積2.106 mで`escape_confirmed=1`となりLowSpeedRejoinへ入った。
- その後、前方車の停止が続いたため2回目のepisodeへ入り、既存step上限でSafeStopした。
- D3はsolver正常のside-wall stallでForward候補を選び、0.049 mしか進めず`forward_duration_limit`となった。本ステアリングのsolver failure対象外である。

### `output/20260718-011435`

- D1はReverseを8 step、累積2.059 m実行して`escape_confirmed=1`となりLowSpeedRejoinへ入った。
- D2は協調Reverseを開始し、0.148 m移動した時点で後方車を検出して停止した。rear hazard発生後にReverseCreepを継続していない。
- D3はWP282付近の連続solver failureから`reverse_only=1`のRecoveryへ入った。
- D3の現在footprintは16 contactを持ち、全Reverse候補が0.05 m先から`contact_worsened`となった。`maneuver_direction_unknown`でSafeStopし、Reverse gear requestは一度も出していない。
- D1 / D2もその後solver unsafeとなり、最終的に全車停止した。

## 判定

「計算不可なら無条件にバック」ではなく、「計算不可が継続し、後方とstatic rolloutの両方が安全な場合だけバック」する実装になっている。協調Reverseと後方車出現時の停止は実走で確認できた。

一方、先頭D3には安全なReverse primitiveがなく、全車停止は解消していない。ここでstatic gateを無効化すると既存接触を深めるため採用しない。次の修正は別ステアリングとし、次を対象にする。

1. 現在contactがあるSide / Mixed状態から、contactを単調に減らせるより短い・細かいReverse primitiveの生成。
2. occupancy map上のfootprint contactとAWSIM実車体接触の対応確認。
3. LowSpeedRejoin直前のsolver fallbackを即時latched SafeStopにするか、Recovery owner中の再評価へ戻すかの整理。

wall証拠なしsolver failure経路はunit testで確認済みだが、今回のdev3でD3が確定した時点ではwall証拠も成立していたため、実走での同経路単独発火は未確認である。
