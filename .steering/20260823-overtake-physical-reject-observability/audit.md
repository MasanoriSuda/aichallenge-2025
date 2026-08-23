# Audit

## Observation before change

- replay: `output/20260823-extended-row-tolerance-replay-v3/d1/autoware.log`
- Overtake canonical fresh shadow:
  - eligible/context/lateral/primal/trajectory: 1136/1136
  - complete: 1130/1136
  - missing 6 cycles: physical certificate rejection
- 1秒窓 `1787491869.029984440`:
  - evaluated=8
  - physical=2
  - complete=2
  - last=canonical-ready

最後の成功周期が詳細を上書きするため、棄却理由は現行artifactから確定不能。

## Hypotheses

1. current pose が既に壁接触・map外であり、保持解でも救えない。
2. current pose から solved horizon への swept connector が壁を横断し、fresh解の初期接続が不成立。
3. horizon stage内の実車footprint sweepが壁へ接触し、QPのFrenet lateral boundsと実空間証明に差がある。

## Falsification

同一bag replayで理由別件数と最終棄却diagnosticを取得し、上記3分類を直接判定する。

## Observation after change

- replay: `output/20260823-overtake-physical-reject-replay/d1/autoware.log`
- source bag: `output/20260823-214300-stop-authority-replay-v2/d1/rosbag2_autoware`
- 物理証明棄却を含む1秒窓:
  - evaluated=27
  - eligible/context/lateral/primal/actuation/trajectory=27/27/27/27/27/27
  - physical/complete=22/22
  - physical reject=5
  - 全5件が `HardWallContact`
  - current pose contact / swept connector / sample unavailable は0
- 最終棄却diagnostic:
  - decision=12157
  - stage=5, waypoint=31
  - current poseから5.970 m先
  - lateral=1.581 m
  - QP bounds=[-0.044, 1.699] m
  - row上のreserve=0.118 m
  - heading offset=0.118 rad
  - physical footprint wall contacts=2

直前replayの6件に対して今回は時刻・起動同期差により5件だったが、全件が同じ1秒窓・同じ理由に集中した。原因分類には十分な再現性がある。

## Conclusion

仮説3を支持し、仮説1・2を反証した。

solver、primal、trajectory extraction、QP lateral rowは成立している。しかし、QP rowが表すFrenet横境界内に点または基準姿勢で収まっていても、予測姿勢を持つ2.0 m x 1.45 mの実車体footprintは壁へ接触する。物理証明が正しくこの不整合を止めているため、証明条件を緩めたりfresh解を強制採用してはいけない。

retained Overtakeをageだけで追加しても根本原因は直らない。次Sliceでは、extended MPCCへ渡すstage-wise lateral rowsを、同じ予測姿勢・車体footprint・壁sampling契約から生成し、QP可行領域と実行時物理証明を一致させる必要がある。

## Validation

- `git diff --check`: pass
- `make autoware-build`: 25 packages pass
- `colcon test --packages-select multi_purpose_mpc_ros`: 40/40 programs pass
- `colcon test-result --verbose`: 1693 tests, 0 errors, 0 failures, 0 skipped
  - 既存の `joycon_contract_guard/package.xml` 欠損警告は残るが、今回の変更とは無関係
