# Results

## Summary

復帰途中の一時条件変化を即時の解除不能SafeStopへしない再評価遷移、wall-awareなsolver
reverse-only判定、solver起因episodeの資格ラッチ、Side / Mixed接触の双方向step候補を実装した。
最終`make dev3` runでは修正前に全車停止した時間帯を越えて3台とも走行を継続した。

## Run evidence

### `output/20260718-184701`

- D2は10 step、2.174 mのstepwise離脱後に`rejoin_complete`へ到達し、再走行した。
- D3はRear wallを検出したが、solver evidence-free設定だけで`reverse_only=1`になり、
  `maneuver_direction_unknown`でSafeStopした。
- 対応: wall証拠と姿勢誤差を候補生成前に評価し、wallなしまたは大姿勢誤差だけを
  solver reverse-onlyとした。

### `output/20260718-185619`

- D3は旧停止点WP251を通過して走行を継続した。
- D2はsolver fallbackからstepwise離脱し、再走行した。
- D1はsolver起因episodeの移動でdetector資格を失い、LowSpeedRejoinが同じsolver復旧を待つ
  循環状態から`solver_unsafe`へ入った。
- 対応: solver起因episodeの資格だけをLowSpeedRejoin完了までcoreに保持した。

### `output/20260718-190601`

- wall-aware判定とsolver資格ラッチは動作した。
- D3はMixed接触7 cellに対し、全Reverse候補が0.05 m以内で9 cellへ悪化するため
  `maneuver_direction_unknown`となった。
- D1 / D2も深い接触またはコース外で有意に移動できず、10 step上限へ到達した。
- 対応: reverse-onlyでないSide / Mixed接触について、Forward Straight / Left / Rightも
  Reverseと同じ0.40 m、操舵sample、`RequireImprovement`条件で比較するようにした。
  Forwardは全Reverse候補よりcontact減少数が大きい場合だけ選び、同値はReverseを維持する。

### `output/20260718-191547`（最終binary）

- race開始後約3分間、D1 / D2 / D3はすべて走行を継続した。
- 終了直前:
  - D1: WP217、3.87 m/s、SafetyBrake（front hazard hold中）
  - D2: WP218、3.00 m/s、LowSpeedAvoidance
  - D3: WP244、2.63 m/s、Cruise
- `state=SAFE_STOP`: 0件
- `escape_step_limit`: 0件
- `maneuver_direction_unknown`: 0件
- `rejoin_timed_out`: 0件
- `forward_hazard_appeared`: 0件
- 接触Recovery自体は0件であり、双方向contact候補がForwardを選択する実走分岐は未観測。
  今回確認できたのは、修正前の約90秒後の全車停止が再現せず、3台が走行を継続したことまで。

## Verification

- `make autoware-build`: 25 packages succeeded。
- `multi_purpose_mpc_ros` package build: succeeded。
- `test_stuck_recovery_core`: 77/77 passed。
- `test_recovery_footprint`: 34/34 passed。
- Targeted total: 111/111 passed。
- `git diff --check`: passed。

package全体の集約`colcon test-result --verbose`には今回の対象外である既存`test_path_core` failureと、
ローカルstale resultの`joycon_contract_guard/package.xml`欠落が残る。上記2 suiteのXMLは
failures=0 / errors=0を確認した。

## Conclusion

本修正の回帰試験と、全車停止再現runに対する時間帯越えのdev3スモーク試験はPassとする。
ただし深いMixed接触が今回再現しなかったため、Forward接触離脱の実走検証は未完了である。
次に同じ接触が再現した場合は`Stuck recovery maneuver selected: direction=Forward`、
contact減少数、step終端の実測改善を受け入れ証拠とする。安全gateを緩める追加変更は行わない。
