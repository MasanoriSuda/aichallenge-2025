# Requirements

## 目的

追い越し中の姿勢崩れからMPC solver fallbackへ入り、静的に安全な後退候補が
存在しない場合に、Reverse-onlyのままSafeStopを永久反復する現象を解消する。

## 確認事象

`output/20260727-051927/d1/autoware.log`では、P1がヘアピン内側への
ShiftOut中にwp185で停止した。

- `e_psi=1.875 rad`によりsolver由来のReverse-onlyが成立
- 現在footprintはclear、wallはnone
- 全Reverse候補は約1.1 m先の静的collisionで不成立
- forward rejoinは静的にclear
- V2X情報はcompleteかつcorridor clear
- `direction=Unknown -> SAFE_STOP -> aggressive retry`を反復

## 要求

1. solver由来のReverse-onlyは、最初の候補評価までは維持すること。
2. 一度SafeStopへ到達し、Reverse候補が実際に評価済みかつ全て不成立の場合のみ、
   Forward fallbackを解禁できること。
3. Forward解禁には、simulation、aggressive recovery、現在footprint clear、
   wallなし、forward rejoin clear、V2X complete/clear、Boost inactiveを要求すること。
4. 解禁後も実際のForward primitiveを静的rolloutとV2X rolloutで再評価すること。
5. Forward候補が不成立ならSafeStopを維持すること。
6. ROS topic/service/message、評価JSON、通常走行・追い越し判定を変更しないこと。

## 対象外

- MPC solver不収束そのものの調整
- ヘアピン内側追い越し軌跡の調整
- 実車Recoveryの有効化
- 静的地図やV2X情報が不完全な状態での発進
