# Requirements

## Goal

追い越し候補の採用判定と、実際にMPCへ渡す横軌跡の物理壁判定を同じ契約へ統合し、
`Idle -> ShiftOut -> wall rejection -> FollowPrepare` の再試行ループを構造的に禁止する。

## Scope

- 左右extended MPCCが物理検証した実軌跡をMissionへ証明書として結び付ける。
- cached Missionを含む新規entryは、現在位置・現在壁境界で証明書を再検証してから採用する。
- Missionのfreeze成功前にFSMをShiftOut/Passへ遷移させない。
- 横境界の交差を中央線`0 m`へ置換せず、型付き不成立としてsolverへ渡さない。
- entry拒否と横境界不成立をdecision ID、target、side、stage、理由付きで記録する。

## Constraints

- 追い越し戦術、速度、壁余裕などのパラメータは変更しない。
- 壁接触、target overlap、EmergencyBrakeの判定を緩和しない。
- ROS 2 topic/service、評価成果物、提出インターフェースを変更しない。
- `output/`、rosbag、result JSON、ユーザーの未コミット変更を編集しない。

## Definition of Done

- 物理検証済みbranchと実行Missionが同じ実軌跡を共有する。
- stale、不足、壁不成立の証明書ではIdleから遷移しない。
- 横境界交差時はsolverを起動せず、構造化理由を残す。
- pure contractの回帰テストと対象packageのbuild/testが通る。
