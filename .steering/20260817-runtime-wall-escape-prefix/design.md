# Design

## Local wall-escape prefix

centerward goalの幾何判定は既存のnominal -> physical clearance fallbackを再利用する。
変更するのは候補の検証範囲である。

従来:

```text
current pose -> contracted goal -> Pass hold -> Return
```

変更後:

```text
current pose -> contracted goal -> short hold
                                  |
                                  +-> rolling replanが残りPass/Returnを更新
```

prefixは既存`shift_distance`を横遷移距離として使い、その後0.5〜1.0 mだけgoalを保持する。
wall bounds、static map、横加速度はこのprefix全体で検証する。候補へ保存するPass残距離は
従来MissionのReturn開始位置を基準に再計算し、置換によってPass距離を二重加算しない。

生成したFrenet DP pathには`frenet_dp_prefix_bridge=true`を設定する。これは現在状態から
短い退避経路だけを所有し、その先を完全Missionとして保証しないことを示す。target ID、
side、Pass累積時間・距離、front-cap releaseはtransactional Mission replacementで維持する。

## Failed-prefix handoff

`RuntimeWallPreplanRequest`へ`center_contraction_evaluated`を追加し、次を区別する。

- fallback delay前でまだ評価していない: fresh same-side候補を待てる
- 評価済みでprefixなし: 現行経路を保持してはいけない
- prefix採用直後のcooldown中: 採用済み補正を短時間実行する

評価済み不成立、またはwall replan回数上限到達時は`ExitCurrentMission`を返す。controllerは
まずdynamic Mission waitへ移り、反対側を含む候補探索を再開する。wait admissionが成立しない
no-return局面ではRecoveryへ移り、壁へ向かう固定goalを解除する。

## Logging

- prefix採用: `runtime wall escape prefix accepted`
- prefix不成立: `runtime wall escape prefix unavailable`
- ログにはgoal変化、prefix距離、clearance種別、handoff先、棄却理由を含める。

## Compatibility

新しい設定キーは追加しない。既存のwall warning、最大centerward adjustment、shift distanceを
利用する。ROS 2インターフェースと提出物契約への影響はない。
