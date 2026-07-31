# 設計

## 現行課題

gap plannerは各予測点で車両を膨張し、壁との間に残るego中心の安全区間を生成している。一方、OvertakeLineへ渡す固定目標は最初の安全区間の中央を基準としており、基準ラインが既に安全でも不要な横移動が発生する。

## 変更方針

通常Overtakeの新規entryに限り、予測区間中の有効な安全区間を交差させた固定区間`[lower, upper]`を使う。

```text
goal = clamp(base_racing_line_ey=0, lower, upper)
shift = abs(goal - current_ey)
```

- `0`が区間内: `goal=0`、`base_line_clear=true`
- `0`が区間外: 0に近い区間端を`goal`にする
- 区間が不正またはpreflight不成立: candidateを実行不能にする

基準ラインが安全でも現在位置が同区間外なら、直接Passにはせず`goal=0`への最小ShiftOutを行う。これにより、追従誤差を理由に膨張済み障害物区間を横切るdirect entryを防ぐ。

左右選択は実行可能候補だけを対象に、次の辞書順とする。

1. `base_line_clear=true`
2. `shift`が小さい
3. 同値なら既存preferred side

`base_line_clear`かつ現在位置も安全区間内で新規entryする場合は`Idle -> Pass`とし、ShiftOutの距離完了待ちを省く。ターゲットlock、front-overlap latch、Return、物理安全guardは通常Passと同じものを使う。

安全候補がない場合はBehavior FSMがFollowを返すため、新規OvertakeLineを生成しない。既に開始したmaneuverのRecoveryは従来どおり残す。

## 対象

- `v2x_overtake_core.hpp/.cpp`: 最小横目標と最小移動side選択の純粋関数
- `mpc_controller_cpp.cpp`: candidate評価、side選択、direct Pass entry
- `config/config.yaml`: rollback用feature flag
- `test/test_v2x_overtake_core.cpp`: 境界条件の単体テスト
