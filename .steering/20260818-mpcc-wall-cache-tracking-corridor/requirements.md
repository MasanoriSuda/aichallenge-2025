# Requirements

## 目的

`output/20260818-091414` で確認した、Pass 中の control callback / OSQP 負荷増加と、
物理壁 envelope cache が実走で再利用されていない問題を解消する。

## 変更範囲

- 静的壁の横方向探索を「Mission 固有の preferred interval」ではなく、waypoint ごとの
  collision-free component として再利用する。
- heading は保守的な角度 bucket と footprint guard を使い、安全側に量子化する。
- cache hit/miss、実走査数、調査した pose 数を既存の1秒周期debugログへ追加する。
- Pass で横離隔を獲得し、現在・予測車体が非重複な間は、相手制約を低レベル tracking
  MPC の hard boundへ二重適用しない。上位のMPCC trajectory生成・物理再検証・壁boundは維持する。

## 制約

- 実壁接触、壁margin違反、現在車体重複、予測sweep重複、EmergencyBrakeは緩和しない。
- 最終のheading-aware lateral profile検証を省略しない。
- ROS 2 topic/service、提出インターフェース、加減速度・clearance設定は変更しない。
- ユーザー変更中の`config.yaml`と`result-summary.json`は変更・コミットしない。

## Definition of Done

- preferred offsetを変えても同一の静的壁scanを再利用できる。
- heading bucket内の差はfootprint拡張で保守的に吸収される。
- Pass target-bound releaseのhard guard条件に単体テストがある。
- package test/buildが成功する。
