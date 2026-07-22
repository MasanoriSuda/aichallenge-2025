# Task list

- [x] 現状のstart-grid、gap planner、OvertakeLine状態を調査する
- [x] 要件と設計を記録する
- [x] 占有区間から境界車両IDを自由回廊へ伝搬する
- [x] スタート時の3回廊選択と連続距離判定を実装する
- [x] 前後にずれた2台を共通Frenet横座標へ投影する
- [x] 2車境界ロックと両車rear-clear判定を実装する
- [x] YAML設定・起動ログ・仕様書を更新する
- [x] 単体テストを追加する
- [x] `make autoware-build` と対象テストを実行する
- [x] 通常速度の短時間`make dev3`で既存breakoutと候補診断ログを確認する
- [x] 20/40/40条件で側方・直後車が境界候補から落ちる問題をstart-grid専用lookbehindで修正する
- [x] Ready物理発進からdomain別StartまでV2X planning sessionを連続させる
- [x] 壁-車候補へ姿勢不明時の矩形包絡と全壁clearanceを適用する
- [x] 壁margin適用後に閉じる回廊を選択前に棄却する
- [x] 車間回廊失敗後のstart-grid未検証side fallback再進入を禁止する
- [x] 中央加速を維持する動的観測とinside/weave/outside確定を実装する
- [x] 操舵限界を超えるwall-side inside候補を棄却してoutsideを再計画する
- [x] inside実行線のwall bound追加余裕を0.8 mへ調整する
- [x] committed Pass中もrear-clear確定を優先してReturnへ移す
- [x] rear-clear済みtargetのReturn即reacquireを禁止する
- [x] occupancy mapとego矩形でOvertakeLineの壁側目標をreference path側へclampする
- [x] occupancy clampのclear / adjusted / invalid条件へ単体テストを追加する
- [x] 動的観測時間・候補安定・offset曲率の単体テストを追加する
- [ ] 実際に2台が12 m内へ残るStart条件で`vehicle-vehicle`ロックを確認する

## Verification notes

- `make autoware-build`: 成功、25 packages。
- `colcon test --packages-select multi_purpose_mpc_ros`: 580 tests、error/failure/skipなし（動的観測・offset曲率テスト追加後の直近結果）。
- `output/20260722-224646`: 通常速度ではD1のStartがD3より約4.5秒遅く、D3が12 m対象外まで先行したため、D1は正しく壁-車の2候補を評価した。
- `output/20260722-224916`: D2/D3を一時1 km/hとして車列を保持したが、3台はReady止まりとなり、`/admin/awsim/start`購読者も不在だったためStart後ロックは未確認。試験後にD2/D3は0 km/hへ復元した。
- `output/20260722-230340`: domain速度20/40/40ではP1のStart時に`vehicles=2/front=1/side=1`だが、旧実装は側方車を後方除外しwall-vehicle候補2本だけになった。start-grid専用4 m lookbehind追加の再現条件。
- `output/20260722-231342`: 4 m lookbehindだけでは各domainのStart受信差までに初期配置が崩れ、P1/P2ともwall-vehicle候補2本のまま。Ready planningが必要と確認。
- `output/20260722-231926`: Readyからplanning sessionを有効にした20/40/40再試験で、P1に`wall-vehicle;vehicle-vehicle[d2,d3];wall-vehicle`の3候補が出現。車間中心`-1.22 m`に対しP1は`e_y=0.94 m`、近い外側中心は`+2.35 m`だったため、最短横移動ルールにより外側を選択。車間候補生成の効果は確認済み、実際のvehicle-vehicle lockは車間が最短になる配置で引き続き確認する。
- 同runの外側raw幅1.65829 mは、target/ego矩形包絡とego-wall clearance適用後に0.2 m未満となる。回帰テストで外側を棄却し、車-車残余幅0.273079 mを維持することを確認した。
- `output/20260723-000738`: P1は車間回廊`e_y=-1.22 m`の消失後、通常fallbackで`e_y=-4.09 m`へ再進入し、hard curve継続によって`e_y=-3.46 m`で壁接触した。start-grid中の未検証fallback禁止を追加した。
- 修正後の`make autoware-build`: 25 packages成功。`colcon test --packages-select multi_purpose_mpc_ros`後の対象結果は577 tests、error/failure/skipなし。
- Ready/session/lookbehind関連の対象GoogleTest 34件（V2X 5件 + start-grid grace 29件）は全件成功。
- 動的観測追加後も`make autoware-build`は25 packages成功。動的候補の安定待ち・最大待ち・緊急確定と、inside offset曲率の操舵限界を含む580 testsは全件成功。
- `output/20260723-060039`: P1は車間Recovery後に通常wall-vehicle線`goal=-4.13 m`へ入り、target rear-clear後もcommitted behaviorがReturn判定を迂回して17秒以上Passを維持した。rear-clear優先Return、rear-clear済みtargetの再取得禁止、wall clearance 0.8 mを適用した。
- 上記状態遷移修正後も`make autoware-build`は25 packages成功、`multi_purpose_mpc_ros`は580 tests、error/failure/skipなし。
- `output/20260723-062353`: `wall_clearance=0.80`適用済みでもP1はPassの`first_target=-3.94 m`を追い、`e_y=-3.916 m`でoccupancy grid上32 cellの壁接触となった。平滑化`lb/ub`と実壁の不一致を確認したため、停止復帰用occupancy gridと車体矩形をOvertakeLine目標検証へ接続した。
- occupancy clamp追加後の`make autoware-build`は25 packages成功。`multi_purpose_mpc_ros`は589 tests、error/failure/skipなし（既存の欠落した`joycon_contract_guard/package.xml`についてtest-result収集warningのみ）。
