# Requirements

## Goal

直近走行で確認された追い越し失敗を、設定値の追加調整ではなく実行契約の整合と構造化ログで切り分け可能にする。

## Scope

- 前方安全距離と、横分離前の接近速度保護距離を同じ計算へ統合する。
- 物理壁検証済みだが追加のQP境界余裕だけが不足する左右branchを、未評価・solver失敗・壁失敗と区別する。
- SafeSeparationのabsolute budget到達時に、既に開始済みの局所forward windowを条件どおり扱う。
- solver fallback停止時に、crawlが使えなかった理由を最終制御ログへ残す。
- 既存ROS 2 topic/service、評価成果物、提出インターフェースは変更しない。

## Constraints

- `output/`、rosbag、result JSONを編集しない。
- 壁接触、target overlap、emergency brakeを緩和しない。
- 新しい戦術・Recovery機能は追加しない。
- ユーザー所有の未コミット変更を変更・削除しない。

## Definition of Done

- 前方安全包絡に単体テストがある。
- branch採否理由とboundary fallbackに単体テストがある。
- SafeSeparationのabsolute/local window境界に回帰テストがある。
- solver crawlの非採用理由に単体テストがある。
- 対象packageがbuild/testを通る。
