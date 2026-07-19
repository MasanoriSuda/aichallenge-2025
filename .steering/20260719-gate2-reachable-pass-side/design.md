# Design

## Root cause

`output/20260719-155127`では初期yawをMPC trajectoryへ一致させて
`e_psi=0.004 rad`まで改善しても、局所plannerは`e_y=+2.031 m`から
`target_ey=-2.71 m`の右側を選んだ。選択基準がgap幅だけだったため、約4.74 mの
横移動を5.55 mのapproach区間で要求し、最初のMPCから解なしになった。
左右選択を到達距離優先にしても左側が最低幅を満たさなかった。原因は低速回避専用の
`v2x_low_speed_avoidance_min_gap_width=0.5 m`ではなく、一般走行用
`gap_min_width=1.8 m`との最大値を使っていたためで、専用閾値が実質無効だった。
さらに、まだ右回廊へ入っていない時点で将来stateを右回廊へhard制約していた。

## Approach

1. 低速停止車回避では専用の最低幅を使い、一般走行用gap幅で上書きしない。
   壁marginの0.8 mから0.2 mへの緩和も切り分けたが、右側目標をさらに外へ広げるだけで
   左側は成立しなかったため不採用とする。
2. 両側feasibleなら、現在`e_y`から各pass targetまでの横移動量が小さい側を選ぶ。
3. 横移動量が同等の場合だけ従来の幅優先を使う。
4. 一度選択したsideは既存lockで固定し、周期ごとの左右振動を防ぐ。
5. egoが選択回廊へ入るまではbase corridorと現在`e_y`を含むshift corridorを使い、
   pass targetをcostとして追従する。実測`e_y`が選択回廊へ入った後だけhard boundへ移行する。
6. Shift中は専用の1.0 m/s上限を適用し、操舵rateが横移動に追いつく時間を確保する。
   回廊進入後は既存の低速回避速度へ戻す。
7. Softなshift targetはsolverが安定したramp ratio 1.0を維持する。0.2と0.5は目標変化が
   急すぎてOSQPが初期状態から失敗した。
8. 左右候補ごとの最小幅と共通区間をログへ出し、候補を落としている制約を数値で確認する。
9. Gate2では左側の物理回廊がなく右側への横移動が必要だった。通常走行のQは変えず、
   低速回避開始後はreference curvature、横偏差、heading偏差を使うbounded feedbackで
   1.0 m/sの直接操舵を行う。heading costの0.1倍と0.5倍はどちらも初期周期から
   OSQPが収束しなかったため不採用とする。
10. `e_y` / `e_psi`が整っただけでは車列途中でMPCへ戻り得る。共通コース進捗上の車両を
    `v2x_low_speed_avoidance_clear_distance`内で車列clearanceとして追跡し、front、side、
    clearance車両がすべて消えてから2秒継続した場合だけMPCへ戻す。

## Compatibility

- ROS topic/service/message契約は変更しない。
- shift速度、横/heading feedback gain、姿勢許容値、連続clear時間は任意設定である。
- 通常走行のMPC重み、ROSインターフェース、評価結果schemaは変更しない。
- 変更は参加者package内に閉じる。

## Result

- `make autoware-build`: 成功
- `V2XOvertakeCoreSide.LowSpeed*`: 11件成功
- `make gate2`: `output/20260719-171944`で成功
- `aichallenge/safety-gate-result.json`: `all_passed=true`, `test2.passed=true`
- 回避制御は`e_y=-2.71 m`, `e_psi=0.00 rad`でMPCへ復帰し、OSQP failureは発生しなかった。
