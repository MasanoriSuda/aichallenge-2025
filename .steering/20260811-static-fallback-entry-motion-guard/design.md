# Design

## 原因

既存static fallbackは、dynamic horizonが候補経路へ届いていない間も追い越し候補を
生成するために必要である。一方、許容区間が静的な壁bounds全体になるため、targetの
横移動とコース曲率が変化している場面でも、コースを横断する大きなShiftOutを生成できる。

全Missionの静的preflightは開始時点の経路を検証するが、動的な相手位置や短時間後の
実行姿勢変化までは保証しない。今回、開始時にはwall clearance 1.02 mと評価された経路が、
約1秒後のlive判定ではcollisionになった。

## 方針

`StaticFallbackEntryMotionAdmission` をpure core policyとして追加する。

次をすべて満たす候補にだけ横移動上限を適用する。

- guard有効
- 新規Mission entry
- corridor sourceが`StaticWallFallback`

上限内なら従来どおり候補へ追加する。上限超過ならそのgoal candidateだけを棄却し、
candidate generatorは次の小さいgoal、別shift distance、反対側の探索を続ける。

dynamic observationで検証された候補は上限対象外とする。active Missionのsame-side／
cross-side再計画も既存のatomic replacement guardを維持するため対象外とする。

## 初期値

```yaml
v2x_overtake_line_static_fallback_entry_motion_guard_enabled: true
v2x_overtake_line_static_fallback_max_entry_lateral_shift: 1.5
```

最新走行では、正常完遂が0.68 mと1.21 m、失敗が3.08 mだった。1.5 mは成功候補を
保持しながら、dynamic未観測状態でのコース全幅級横断を除外する初期A/B値とする。

## 非対象

- static fallbackの全面撤廃
- active Pass／Returnの経路変更
- wall margin、加速度、closing speedの変更
- solver recoveryの変更

## 動的確認

- 失敗地点で`static_fallback_entry_motion_rejected`が増え、3.08 m候補を選ばないこと。
- より小さい同側または反対側candidateを選ぶか、Followを継続すること。
- 正常例の0.68 m／1.21 m級static fallbackが引き続き完遂すること。
- `locked target stale or lost -> solver_unsafe -> Reverse`が再発しないこと。
