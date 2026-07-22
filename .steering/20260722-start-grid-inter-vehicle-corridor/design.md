# Design

## Current problem

既存のstart-grid breakoutは最寄り前車を1台だけ選び、その車の左か右を決める。`v2x_vehicle_vehicle_gap_enabled` を有効にしても通常設定の距離18 m・幅3.2 mで車間候補が除外され、候補を選べた場合も両側車両IDを保持しない。このためP1/P2が空いている車間へ入らず、単一前車の後ろへ吸い込まれる。

## Candidate representation

- 占有区間に、その下端・上端を作った車両IDを保持する。
- 自由区間に下側境界ID・上側境界IDを伝搬する。
- 両境界が車両ならvehicle-vehicle corridorとする。
- planner outputは最初に選んだ回廊種別、境界ID、幅、連続距離を公開する。

## Start-grid selection

スタート猶予中の新規breakoutでは、pass sideを固定せず全自由区間を評価する。前後にずれた2台を各車の最寄りコース線分へ投影し、共通Frenet横座標で3回廊を構成する。選択時だけ2台の長手占有を設定距離まで持続させ、必要幅を満たした候補を、現在の横位置から回廊中央までの距離を主項、幅を副項として採点する。車間候補は同じ境界IDの組が設定距離以上連続する場合だけ採用する。

初期値:

- 最小残余幅: 0.2 m
- 最小連続距離: 3.0 m
- スタート車列の長手占有: 12.0 m
- スタート選択・ロック中の対象車横半径: 1.25 m（予測marginは別加算）

`v2x_vehicle_radius=1.45 m` は通常走行の円形近似として維持する。スタート車間だけ1.25 mへ縮めるのは、ゴム接触を許容した2025 AWSIM予選の攻めた設定であり、実車設定へ流用しない。

## Dynamic launch decision

Ready直後の静止配置では進路を固定しない。P1は中央trajectoryとdomain速度参照を維持して加速し、
いずれかのpeer速度が0.25 m/sへ達してから0.4秒、最大1.25秒だけ回廊を再計算する。同じ
`strategy/side/boundary IDs`が0.2秒継続した時点で確定する。候補は各周期のV2X予測位置・速度を
使うため、P2/P3が作った隙間に応じて`inside / weave / outside`が変わる。観測中のbehaviorはCruise
として、Follow速度制限と横回避目標を適用しない。EmergencyBrake相当なら観測を打ち切り、実在する
候補へ即時commitし、候補がなければ既存SafetyBrakeへ渡す。

壁-車のinside候補は、固定Frenet offsetの曲率`kappa/(1-kappa*e_y)`を予測区間で計算する。
必要曲率が`tan(delta_max)/wheelbase`を超える、またはFrenet分母が0.1以下になる候補は棄却し、
反対側のoutsideを明示的に再計画する。車-車のweaveは両車rear-clearまでの専用契約を持つため、
このwall-side inside判定の対象外とする。

insideを採用できても、固定横目標はヘアピンで変化するwall boundに対して再clipされる。
`output/20260723-060039`では通常wall-vehicle線の固定`goal=-4.13 m`をrear-clear後も17秒以上
Passで保持したため、WP62まで壁側目標を出し続けた。実行余裕を0.8 mへ戻し、Pass中の
rear-clear確定をbehaviorのcommitted continuityより優先してReturnへ移す。同一targetが既に
rear-clearならReturnからの即reacquireも許可しない。候補回廊の0.2 m条件は変更しない。

`output/20260723-062353`では0.8 mのwall bound余裕を読み込んだ後も、P1のPass目標
`e_y=-3.94 m`と実車位置`-3.916 m`が停止復帰用occupancy grid上の壁へ32 cell接触した。
これは平滑化された`lb/ub`が実壁位置を表していないため、数値marginを増やすだけでは防げない。
OvertakeLineの各horizon目標について、同じoccupancy gridへego矩形を置き、左右extentだけを
`min_wall_clearance`だけ膨張して検証する。接触する目標は固定pass側からreference path側へ
0.05 m以下の刻みで戻し、最初に成立した横位置へclampする。0.8 m余裕が物理的に入らない場合は
追加膨張なしの車体接触回避へ落とし、それも成立しなければreference path目標とRecovery速度上限を使う。
前後extentは追加膨張しないため、ヘアピンの前後壁を横marginとして二重に数えない。

## Wall-corridor geometry

V2X messageにはtarget yawがないため、1.25 mの円形近似を壁-車候補にも流用すると、
斜め車体と壁の間に存在しない回廊を生成する。壁-車候補ではegoとtargetの矩形半対角を
最大横半幅として使い、車両側境界を追加膨張する。壁側もego矩形半対角以上をcenter clearance
として確保する。車-車候補にはこの追加膨張を適用せず、1.25 mの攻めた設定を維持する。

自由区間は車両膨張と壁clearanceをすべて適用した後の残余幅でfilter・採点する。
旧処理のように壁clearanceを区間幅の半分へclampしない。完全に閉じる回廊はその時点で棄却する。
`output/20260722-231926`の外側候補は、旧raw幅1.65829 mから矩形追加膨張と壁clearanceを
差し引くと0.2 m未満になり、車-車候補0.273079 mだけが残る。

## Lock and continuation

車間回廊選択時、`OvertakeLineState` に下側・上側境界IDと固定中央を保存する。実行中gap plannerは同じIDの組だけを有効候補として、スタート車間専用横半径で再検証する。長手占有の12 m化は選択時だけで、実行中は実際の予測位置と車両長を使う。通常の単一target IDは既存の速度制御・診断との互換性のため維持する。

Return判定は通常追い越しでは従来通り単一target rear-clear、車間回廊では両境界rear-clearのANDとする。片側欠落は短い既存target hold中は保持し、回廊再検証が継続して失敗した場合はRecoveryへ移る。

`output/20260723-000738`では、車間目標`e_y=-1.22 m`が実行中に無効となってRecoveryした直後、
start-grid例外のまま通常side-clearance fallbackへ再進入し、target相対の`e_y=-4.09 m`を固定した。
この未検証fallbackはstart-grid中だけ禁止する。回廊消失時はRecoveryで中央へ戻り、同じスタート
例外で新しい固定イン目標を作らない。通常周回のfallbackは維持する。

## Configuration

`V2XBehaviorConfig` にstart-grid専用の次を追加する。

- `v2x_start_grid_inter_vehicle_corridor_enabled`
- `v2x_start_grid_inter_vehicle_min_gap_width`
- `v2x_start_grid_inter_vehicle_min_open_distance`
- `v2x_start_grid_inter_vehicle_longitudinal_span`
- `v2x_start_grid_inter_vehicle_lookbehind_distance`
- `v2x_start_grid_inter_vehicle_lateral_radius`
- `v2x_start_grid_dynamic_decision_enabled`
- `v2x_start_grid_dynamic_peer_motion_speed`
- `v2x_start_grid_dynamic_motion_observation_sec`
- `v2x_start_grid_dynamic_max_observation_sec`
- `v2x_start_grid_dynamic_candidate_stable_sec`

20/40/40のdomain速度差ではReady中に高速側が先へ動き、P1がStartを受ける時点で境界車の1台が側方・直後になる。通常front判定とは分離したstart-grid専用lookbehind窓で、その車両を車-車回廊の境界候補に残す。

さらにAWSIMはReady中に物理走行を開始し、各domainのStartはスタートライン通過時に数秒ずれて届く。V2X planning sessionをPreparedから有効にし、Readyで選択したbreakout lineをStartまで連続保持する。

通常の `v2x_vehicle_vehicle_gap_min_distance/min_width` は変更せず、スタート専用例外を明示的に分離する。

## Diagnostics

選択時に回廊種別、pass side、中央、幅、連続距離、両境界IDをINFOログへ出す。OvertakeLine debugにも車間ロックの有無とIDを出す。occupancy clampの作動、要求marginの縮退、物理車体でも不成立の3状態も分離して出す。
