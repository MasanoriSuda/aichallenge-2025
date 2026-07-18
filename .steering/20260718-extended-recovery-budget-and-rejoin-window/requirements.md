# Extended Recovery Budget and Rejoin Window Requirements

作成日: 2026-07-18
状態: Completed (Rejected)

## 背景

`output/20260718-164220`のD3はSide / Mixed contactから適応操舵でcontactを242から100へ
低減したが、10 step・episode実移動1.076 mで`escape_step_limit_reached`となった。各候補は
contact改善を予測し、runtime悪化時は直ちに停止しており、距離・static・V2X gateには余裕がある。

同runのD2は2.006 m離脱後にLowSpeedRejoinへ入り、5秒で横偏差を-1.133 mから-0.832 mへ
改善したが、許容0.5 mへ入る前に`rejoin_timed_out`となった。

## 要求

1. 最大escape stepを10から16へ増やし、改善が継続する深いcontactの追加離脱機会を評価する。
2. 最大後退距離3.0 m、単step 0.40 m、最大速度0.8 m/s、単step時間4.0秒を変更しない。
3. runtime contact悪化時は即停止し、static swept footprint、V2X completeness、rear corridor、Boost、gear gateを維持する。
4. LowSpeedRejoin timeoutを5.0秒から10.0秒へ延長し、横偏差の収束継続を評価する。
5. Rejoin速度1.0 m/s、static lookahead 0.8 m、横偏差0.5 m、heading誤差0.35 rad、確認0.3秒を変更しない。
6. solver、rejoin static、V2X情報、gearが不安全な場合のSafeStopを維持する。
7. ROS topic、service、message、Domain、評価JSON契約を変更しない。

## 2025 AWSIM向け実験値

- `max_escape_steps: 16`
- `rejoin.timeout_sec: 10.0`

これらは2025 AWSIM final_ver3用のローカル実験値であり、2026公式値・実車値ではない。

実験結果が採用条件を満たさなかったため、実運用設定には残さない。

## Definition of Done

- YAML変更後に`make autoware-build`が成功する。
- `make dev3`でD1〜D3のcontact、step、episode距離、LowSpeedRejoin、最終状態を記録する。
- D3のcontactが10 step時点以降も減少するか、clearになるかを確認する。
- D2が10秒以内に`rejoin_complete`するか、別の安全理由で停止するかを確認する。
- contact悪化中の継続駆動、3.0 m超過、V2X不完全時の後退をFailとする。
