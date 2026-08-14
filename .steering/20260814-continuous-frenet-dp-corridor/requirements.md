# Requirements

## 目的

現行の固定 `pass_lateral` Missionでは、各予測時刻の走行可能区間に共通部分がない場合、
横位置を連続的に変えれば通過できる状況でも `planning_unavailable` または
`physical target separation conflicts with wall bounds` になる。

Proレビューで参照された Liniger MPCC の構成に寄せ、次の二段構成を導入する。

1. Frenet時系列回廊を離散DPで評価する。
2. 選択したhomotopy内は既存のreceding-horizon横軌道optimizerで連続最適化する。

## 必須要件

- 左右の動的free-corridorを時系列として評価する。
- 前回経路、横移動量、回廊幅、side変更を同一コストで扱う。
- 固定横目標の共通部分がなくても、連続DP経路がある場合は短期prefix候補を生成できる。
- DP由来候補を完全rear-clear Missionとして扱わない。
- DP解がない、古い、数値不正の場合は既存候補生成へfail closedする。
- 壁、車体、横加速度、body-clear、V2X continuity、no-returnのhard guardを迂回しない。
- `/control/command/control_cmd` 等のROS 2インターフェースを変更しない。

## 非対象

- Liniger MPCCやEVO-MPCCコードの直接移植
- solverの全面置換
- tactical plannerの別process化
- Recoveryアルゴリズムの変更
- Return以後の経路最適化

## Definition of Done

- DP corridor coreの左右対称、連続性、前回経路保持、不成立テストが通る。
- 固定goal不成立・DP経路成立時にprefix-only候補となるテストが通る。
- `multi_purpose_mpc_ros` がビルドできる。
- package testが0 failureで完了する。
- 実走ログでDPの選択side、cost、prefix bridge使用有無を確認できる。
