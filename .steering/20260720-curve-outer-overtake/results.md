# Results

## 実装結果

- 曲率内側符号と反対側を外まくり候補として判定するpure coreを追加した。
- soft curveでは、外側gap成立時だけ新規ShiftOutのcurve/completion入口guardを緩和する。
- hard curve内で新規開始は許可せず、開始済み外側ShiftOut/Passだけを継続する。
- hard継続にはlocked target観測と外側gap継続を要求する。
- イン差し、明示禁止WP、curve cooldown、EmergencyBrakeは従来どおり拒否する。
- debugへ`outer_entry`と`outer_hard`を追加した。
- dev3 configで次を有効化した。

```yaml
v2x_overtake_outer_curve_entry_enabled: true
v2x_overtake_outer_curve_hard_continuation_enabled: true
```

## 検証

### YAML / diff

```text
config.yaml: OK
git diff --check: OK
```

### V2X pure core

```text
[==========] Running 100 tests from 15 test suites.
[  PASSED  ] 100 tests.
```

追加した6ケースはsoft curve外側進入、inner/hard新規進入拒否、hard継続、
gap/locked target要求、明示禁止/cooldown/Emergency guard、無効時の従来互換を確認する。

### Build

```text
make autoware-build
Summary: 25 packages finished [7.58s]
[build_autoware] Build successful.
```

stderrは既存の`setup.py install is deprecated`警告のみ。

最初の`colcon test-result --verbose`は、今回の対象外である既存`test_path_core`の
`RemovesOneEndpointFromConfiguredFinalVer3Trajectory`失敗と、欠落した
`joycon_contract_guard/package.xml`を過去のtest resultから拾って終了コード1になった。
対象`test_v2x_overtake_core`を再実行し、100/100成功・終了コード0を確認した。

## dev3確認項目

- soft curve進入で`outer_entry=1`となり、外側符号のShiftOutへ入ること。
- 内側候補では`outer_entry=0`のままであること。
- hard curveへ入った開始済み外側ラインで`outer_hard=1`となること。
- hard curve内から新規ShiftOutを開始しないこと。
- 外側gapまたはlocked target消失時はRecoveryへ戻ること。
- rear-clear確認前にReturnせず、外側で前走車を抜き切ること。
- solver failure、壁接触、SafetyBrake回数が増えていないこと。
