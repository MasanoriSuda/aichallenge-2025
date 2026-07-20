# Results

## 実装結果

- moving-front Follow capを固定`front speed + 0.8 m/s`から中心間距離連動へ変更した。
- 比較実験用dev3設定では2.3〜4.4 mで`front speed - 0.6 m/s`、5.0 mで同速、5.8 m以上で従来の
  `front speed + 0.8 m/s`となる。
- 2.3 m以下のmoving frontは相対速度が小さくてもSafetyBrakeとした。
- 距離回復域はFollowだけでなく、横クリア前のShiftOut/Passにも適用した。
- 横方向のgap、壁margin、OvertakeLine、target hold値は変更していない。
- debugへ`clearance_cap`と`clearance_margin`を追加した。

## 検証

### V2X pure core

```text
[==========] Running 94 tests from 14 test suites.
[  PASSED  ] 94 tests.
```

距離3.0 / 2.5 / 2.0 / 1.0 mの連続cap、旧設定互換、hard distance境界を含む。

この単体テストの距離値は距離連動機構の境界テストであり、dev3の有効設定値そのものではない。

### Build

```text
make autoware-build
Summary: 25 packages finished [1min 17s]
[build_autoware] Build successful.
```

stderrは既存の`setup.py install is deprecated`警告のみ。

## `output/20260720-005937`の再分析

初回設定（hard 2.0 m、target 2.5 m、recovery 0.5 m/s）では、
`output/20260720-004016`の継続接触は大きく改善した。

- P1 moving-front: 133 samples、minimum 1.77 m、mean 2.74 m、2.0 m未満1 sample
- P2 moving-front: 141 samples、minimum 2.43 m、mean 2.84 m、2.0 m未満0 sample
- 修正前はP1が81 samples中65、P2が87 samples中60で2.0 m未満だった

一方で、P1は2.5 m未満が24 samplesあり、追い越し継続下限2.5 m付近で
Follow/Overtakeが切り替わっていた。そこでhardを2.3 m、targetを3.0 mへ広げ、
recoveryを0.6 m/sへ上げる。横gap、壁margin、OvertakeLine、close-follow設定は変更しない。

### 追調整後の静的検証

```text
config.yaml: OK
git diff --check: OK
make autoware-build
Summary: 25 packages finished [3.90s]
[build_autoware] Build successful.
```

stderrは既存の`setup.py install is deprecated`警告のみ。dev3実走はユーザー側で行う。

## 5.0 m追走目標の比較実験

- `v2x_moving_follow_target_distance`を3.0 mから5.0 mへ変更する。
- hard 2.3 m、recovery 0.6 m/s、gain 1.0、最大接近速度差0.8 m/sは維持する。
- Follow速度制限開始距離も5.0 mなので、5.0 mより遠方の追い上げは変更しない。
- 中心間5.0 mは、nominalな車体占有長2.0 mを引くと約3.0 mのバンパー間距離に相当する。
- 横gap、壁margin、OvertakeLine、close-follow設定は変更しない。

```text
config.yaml: OK
git diff --check: OK
make autoware-build
Summary: 25 packages finished [3.97s]
[build_autoware] Build successful.
```

## dev3確認項目

- `fd <= 2.3`で`danger_action=SafetyBrake`、reasonが
  `moving front inside hard center distance`になること。
- `2.3 < fd < 5.8`で`clearance_cap=1`となり、`clearance_margin`が
  `-0.6`から`+0.8`へ連続的に変化すること。
- 継続接触せず、中心間距離がおおむね5.0 mへ回復すること。
- 追い越し開始が遅すぎず、5.0 m付近から横ShiftOutへ入れること。
- 横クリア後は`clearance_cap=0`となり、Pass加速が再開すること。
- `ShiftOut -> Pass -> Return`の発生回数とSafetyBrakeの持続時間を確認すること。
