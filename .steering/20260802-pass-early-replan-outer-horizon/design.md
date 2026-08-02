# Design

## 1. Early rear-clear replan

現行はstatic/dynamic horizon残距離が3 m以下、残時間が0.75秒以下、または予測重複時に
same-side extensionを要求する。しかしreplacementの距離上限12 mより遅く要求されるため、
現在位置からrear-clearまでの必要距離を収められない。

Pass中は最新target状態、現在速度、missionのオフセット曲率速度上限を使って、現在位置から
rear-clearまでの必要Pass終端を再予測する。初回extension前に次を追加トリガーとする。

```text
live required rear-clear Pass end
  > currently validated static Pass end
  -> RequestSameSideExtension
```

現在missionでrear-clear可能な場合は再計画しない。相手との実際の前後進捗や曲率速度上限により
当初終端を超えることが予測された時点で、残り3 mを待たずfull replacementを生成する。
予測不能の単発サンプルでは早期Holdへ落とさず、既存のhorizon/overlap判断へ委ねる。
extension成功後は既存の1回上限を維持し、同じトリガーを再発させない。

## 2. Rear-clear-sized replacement

`pass_horizon_extension_max_distance` は早期再計画を始めるnominal windowとして残す。
実際のreplacementは現在Pass位置からabsolute distance limitまでをrolloutし、予測rear-clearに
必要な距離を算出する。必要距離がnominal windowを超えてもabsolute 32 m内であれば、
Shift/Pass/Return全体のstatic preflightを通した上で採用する。

未検証の距離へclampして採用することはしない。absolute limitまたはstatic preflightを満たせない
場合は従来どおり失敗とする。

## 3. Strategic outer continuity

missionのShiftOut開始からpredicted rear-clearまで、基準trajectory上の有意な曲率サンプルを走査する。
最初の有意な曲率に対してpass sideが外側なら、そのmissionをouter strategyとして確定する。

```text
inside side = sign(reference curvature)
outer mission = pass side != inside side at first significant curve

outer mission中に pass side == inside side となる有意曲率が出現
  -> strategic role reversalとして棄却
```

意図的に選ばれたinside attackはこの規則では棄却しない。outer strategyの情報はmission stateへ保存し、
same-side extensionでもrear-clearまで維持する。

## 4. Offset-curvature speed rollout

各候補のmission pathから距離ごとの横位置を求め、次式でoffset経路曲率を計算する。

```text
kappa_offset = kappa_reference / (1 - kappa_reference * e_y)
```

速度上限は基準trajectoryの `v_ref` に加えて、Overtake実行と同じ横加速度上限を使う。

```text
v_curve = sqrt(min(ay_max, overtake_line_max_lateral_accel) / abs(kappa_offset))
```

inside offsetは速度上限が下がり、outer offsetは曲率が緩くなる。これをrear-clear rolloutへ渡し、
曲線内でclosing speedを作れない候補を採用前に除外する。

## 5. Integration points

- `v2x_overtake_core`: early replan actionとouter continuityを純粋関数化する。
- candidate admission: goalごとのoffset曲率speed capsとouter continuityを評価する。
- OvertakeLine state: outer strategy確定状態をmissionとともに保持する。
- same-side extension: absolute残距離でrear-clear rolloutし、outer continuityとfull static pathを再検証する。
- log: early trigger、nominal/required replacement距離、outer role reversal距離を出力する。

## 6. Non-goals

- course-progress予測フラグの有効化
- extension回数の増加
- wall margin、Emergency、current-overlap guardの緩和
- Pass失敗後のSafeSeparation FSM追加

SafeSeparationは別の残課題とし、今回のPass計画成立性修正へ混在させない。
