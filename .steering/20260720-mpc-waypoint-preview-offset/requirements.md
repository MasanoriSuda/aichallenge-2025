# Requirements

- `mpc.wp_id_offset` と `mpc.wp_id_low_offset` を `0..2` の範囲で利用可能にする。
- waypoint offset を変更しても、自己位置の横偏差・方位偏差を現在最近傍 waypoint 基準で計算する。
- 経路制約、V2X 判定、予測軌跡の座標系を現在最近傍 waypoint 基準に維持する。
- offset 先の waypoint は入力目標候補（preview）にだけ使用する。
- preview は早期減速と同方向の早期切り増しに限定し、早期加速・早期切り戻しを禁止する。
- circular path の周回境界と non-circular path の終端を安全に処理する。
- 起動時 YAML と動的 ROS parameter の両方で範囲外値を拒否する。
- 既存のユーザー設定差分を変更しない。
