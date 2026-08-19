# Design

## 方針

Mission を作り直す前に、既に検証済みの横逃げと failover 評価権限を失わない
ようにする。安全条件を新しく緩和するのではなく、同じ周期内で別々に計算
されていた判断結果の authority handoff を明示する。

## 1. Pre-contact escape ownership

`CommittedPassGeometryOwnershipResolution` に pre-contact escape の所有権を追加
する。次を全て満たす場合だけ Pass の横方向所有を維持する。

- pre-contact squeeze classifier が active
- 現在車体は非重複
- overlap confirmation は未成立

front-cap は速度の保護として再適用できるが、それだけで横逃げ軌道を破棄
しない。壁で適用可能な横biasが得られたかは controller state に保存し、次周期
の committed continuity bridge に使用する。

## 2. Cross-side assessment lease

DynamicMissionWait へ入る直前の `before_no_return` を state に latch する。
lease は既存の以下の小さい方の範囲だけ有効とする。

- `v2x_overtake_dynamic_mission_wait_timeout_sec`
- `v2x_overtake_dynamic_mission_wait_max_distance`

target continuity、body separation、prediction、hard-fault、replacement count は
毎周期再検証する。lease は左右候補を評価・採用する権限だけを保持し、候補の
物理 preflight を省略しない。

## 3. Diagnostics

runtime-failover trace に次を追加する。

- `cross_side_lease`
- `escape_authority`
- `current_reason`
- `alternate_reason`

`locked target entered selected pass-side line` は安定カテゴリ
`pass-side-intrusion` として分類する。

Pass から FollowPrepare へ遷移すると Pass 内部 state は初期化されるため、
`escape_authority` は DynamicMissionWait 入口で診断専用 latch へ退避する。
これにより、失効後のログでも「横逃げが開始済みだったか」を判別できる。

runtime-failover の候補表示は次の形式とする。

```text
current=feasible/mission/ready/reason="..."
alternate=feasible/mission/stable/urgent/ready/reason="..."
cross_side=allowed/lease=active
escape_authority=active
```

## 非対象

- OSQP の重み・反復上限調整
- 入口8 m completion reserve の変更
- Recovery/Reverse の変更
- 新しい追い越しパラメータの追加
