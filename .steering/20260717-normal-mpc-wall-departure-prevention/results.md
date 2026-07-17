# 通常MPC壁逸脱予防 Results

実験日: 2026-07-17
run: `output/20260717-234612`
変更: `mpc.steering_tire_angle_gain_var: 1.5 -> 1.0`

## 検証

- `make autoware-build`: 成功（25 packages）
- `make dev3`: 3台起動、対象WP通過後も監視を継続
- `make down`: 成功
- 比較対象: `output/20260717-232948`

## 結果

| 車両 | baseline | 今回 |
|---|---|---|
| D3 | WP72で`e_y=-1.964 m`、current wall contact 171 cells、OSQP failure | WP72とWP90をStart後contact / solver failureなしで通過。1周136.583 s（result 136.673 s） |
| D1 | WP121〜123で大きな横・方位誤差からOSQP failureと壁際停止 | WP123とWP140をStart後contact / solver failureなしで通過。1周142.589 s（result 142.644 s） |
| D2 | 全車停止列の一部 | Start後のwall contactなし。2周目WP34〜41で別のOvertakeLine / solver事象により停止 |

D1とD3は旧runで3台停止列となったD2 Start後約79秒を越え、1周完了後も走行した。Start後の
current wall contactは全domainで検出されず、通常MPCの壁逸脱予防という受け入れ条件はPassした。

## 残課題

D2は2周目のWP34付近でFollowからOvertake ShiftOutへ入り、対象消失後にRecoveryへ遷移した。
WP37以降は断続的なOSQP failureが連続化し、WP41で速度0、solver failure 697回まで継続した。
Recovery中の横誤差は約`-0.39 -> -2.01 m`へ拡大し、solverが復旧していない状態で追い越し判定へ
再進入している。map wall contactはなく、今回解消した通常MPCの過操舵とは別問題である。

次の実験は新しいステアリングを切り、OvertakeLine Recovery中の追い越し再進入禁止、solver fallback
時の安全停止または中央線復帰、復帰完了条件を対象にする。

## 判定

- 通常MPC壁逸脱予防: **Pass**
- dev3全車継続走行: **Partial**（D2の別事象）
- Phase 1設定: **採用**（2025 AWSIM向け暫定値）
