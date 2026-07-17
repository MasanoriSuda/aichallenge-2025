# 通常MPC Failure操舵中立復帰 Results

実験日: 2026-07-18  
実験run: `output/20260718-003725`  
総合判定: Partial（中立復帰Pass / dev3停止解消Fail）

## 実装と静的検証

- 通常MPC solver failureは4周期まで直前操舵を保持し、5周期目から`steer_rate_max`以内で0へ戻す。
- OvertakeLine Recovery / solver re-entry gateでは従来どおり待機せず中立復帰する。
- `make autoware-build`: 25 packages成功。
- `test_v2x_overtake_core`: 44 tests、9 suites、全件成功。

## dev3観測

D3は旧runのWP237をsolver failureなしで通過した後、WP272付近で大きな姿勢ずれを伴う
連続OSQP failureへ入った。

| failure周期 | fallback速度 [m/s] | 操舵 [rad] | mode |
|---:|---:|---:|---|
| 1 | 0.879 | 0.559 | hold |
| 5 | 0.773 | 0.529 | neutralize |
| 10 | 0.604 | 0.379 | neutralize |
| 20 | 0.266 | 0.079 | neutralize |
| 30 | 0.000 | 0.000 | neutralize |

40 Hz、`steer_rate_max=1.2 rad/s`に対応する最大`0.03 rad/cycle`で減少しており、設定した
rate limitを満たす。旧run `output/20260718-000645`で407周期保持された`-0.436 rad`のような
長時間の操舵固定は解消した。

D2のWP147付近でもShiftOut中に8連続failureが発生した。1周期目は`0.023 rad`をholdし、
5周期目には`0.000 rad`へneutralizeした後、8周期でsolverが復旧した。既存の追い越しRecoveryとの
明確な回帰は観測していない。

## 残った停止原因

D3の最初のfailure時点で`e_y=1.044 m`、`e_psi=-1.808 rad`であり、操舵fallbackへ入る前から
車体姿勢がreference pathに対して大きくずれていた。連続failure中に現在wall contactは0だったが、
2秒のfallbackとwall evidenceでStuck Recoveryが成立した。

Recoveryは`forward_left`、目標`0.300 m`を選んだが、`0.123 m`だけ進んだ時点で
`forward_duration_limit`となりSafeStopした。停止時も現在wall contactは0、
`e_y=0.671 m`、`e_psi=-1.513 rad`で、solverが扱える姿勢へ戻せていない。

最終的にD3はWP271でSafeStopし、D1はWP264、D2はWP267で前方車に対するSafetyBrakeとなった。
したがって、全車停止の直接系列は「D3の大きな方位誤差 -> solver連続失敗 -> Recovery距離未達 ->
D1/D2の後続停止」であり、操舵holdだけが原因ではない。

## 基準runの訂正と分離

- `output/20260718-000645`: D3 WP237付近の長時間solver failureと操舵hold。本変更の対象。
- `output/20260718-001009`: D3 WP222でsolver failureなし、`e_y=-1.650 m`とwall contactが発生した
  別事象。本runではD3がWP222を通過しており、本変更の成否とは分離する。

## 採用判断と次の課題

中立復帰は異常時の固定操舵を除去し、接触を増やさず停止できたため採用する。ただしdev3の
デッドロック解消策としては不十分である。次は別ステアリングで、大きな`|e_psi|`を伴う
solver failure時の再配向Recovery、primitive選択、escape距離と時間上限、MPC再合流gateを設計する。
時間上限だけを延長すると壁方向へ駆動する危険も延びるため、単独では変更しない。
