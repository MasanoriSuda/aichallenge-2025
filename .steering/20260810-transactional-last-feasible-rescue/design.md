# Design

## Cache policy

cache 更新を次の3種類に分ける。

- Replace: 新しい complete/preflighted candidate が成立した。
- Retain: 1周期だけ candidate が欠落した、予測 horizon が一時的に不成立などの soft miss。
- Clear: target/generation 不一致、target discontinuity、非回復の車体 overlap などの hard invalidation。

Retain は最大 `0.50 s` のTTL内だけ許可する。

## Freshness

候補保存時に以下をanchorとして保存する。

- ego Mission進行距離
- ego lateral位置
- target longitudinal位置
- target course-lateral位置
- 保存時刻とMission generation

再利用時は age に加え、初期値として以下を要求する。

- ego進行差 `<= 1.50 m`
- ego横位置差 `<= 0.30 m`
- target縦位置差 `<= 1.00 m`
- target横位置差 `<= 0.25 m`
- candidateのdynamic validity期限内

## Transactional replacement

置換は旧Missionをinvalid化せず、次の順で行う。

1. cache identity/freshness/no-returnを検証
2. 現在poseからPassPlanを再構築
3. 現在の実行条件を再検証
4. 現在stateをsnapshot
5. 新Missionをcommit
6. commit後の整合が取れない場合はsnapshotへrollback

同側refreshは旧Missionがinvalid化済みであることを要求しない。置換成功時だけgenerationを更新する。

## Hard fault

以下ではcache rescueを禁止する。

- target discontinuity/position jump
- 回復不能なcurrent body overlap
- wall physical contact/margin violation/sample unavailable
- EmergencyBrake
- solver recovery
- overtake forbidden waypoint

既存のrecoverable side contactはcurrent body overlapの例外とし、横分離しながら前進する既存方針を維持する。

