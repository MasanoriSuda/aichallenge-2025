# Requirements

## 目的

solver bounded continuation 復帰時だけでなく、通常の ShiftOut / Pass / Return 実行中にも、実際に採用される MPC 予測軌道を物理 footprint で再検証し、壁接触予測を含む制御を publish しない。

## 根拠

- `20260820-163839` と `20260820-164400` では solver wall handoff gate により、復帰直後の予測接触は拒否できた。
- 一方、通常の overtake episode は合計 16 回開始して `Pass -> Return` は 0 回だった。
- `20260820-164400` episode 3 は frozen goal 1.20 m に対して `e_y=3.26 m`、wall minimum 0.05 m、推定 lateral acceleration 37.07 m/s2、authority change 27 回を記録した。
- DP execution authority が tracking 判定境界で release / retain を反復し、通常解側の経路採用に共通の最終 wall admission がなかった。

## 制約

- ROS 2 topic、message、launch、提出インターフェースは変更しない。
- config の攻撃度や壁 clearance 値自体は変更しない。
- 物理 wall/contact、emergency、solver recovery は即時 hard fault のまま維持する。
- 40 Hz 全周期で重い footprint scan を追加せず、共通 admission の物理評価は 10 Hz を基本とする。
- ユーザーの既存変更と生成物を変更・コミットしない。

## Definition of Done

- active overtake の unsafe prediction が publish 前に保留される。
- 2 回連続の新しい安全観測まで保留を解除しない。
- DP tracking の単発境界超過では authority を即時 release しない。
- authority release / retain ログに直接の理由が出る。
- gate scope、予測壁距離、hold周期、phase/path sourceが決定ログから追跡できる。
- 対象 package の build と unit test が成功する。
