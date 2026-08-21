# Unified Race MPCC foundation requirements

## Objective

通常MPCと追い越しMPCCの接続部で発生している座標、solver状態、非同期候補、実行権限の不整合を除去し、将来の常時Race MPCCへ安全に移行できる基盤を作る。

本作業はRace MPCCへの即時切替ではない。現行制御を維持したまま、同じ物理・座標契約でLeft / Right / Hold / Returnを評価できる状態までを対象とする。

## Required changes

1. horizonのwaypoint、区間距離、累積距離を一つの`StageGeometry`として生成する。
2. MPC dynamics、壁回廊、MPCC execution path、physical certificateが同じstage定義を使う。
3. 左右extended MPCCはworker内でside別のsolver contextを継続利用する。
4. solver contextはtarget、context epoch、horizon構造が変わったときだけ明示的にresetする。
5. physical certificateへtarget観測のtimestamp、course progress、course lateral、prediction generationを含める。
6. async result採用時に現在targetとの差を再検証する。
7. Left / Right / Hold / Returnを共通形式で表現するRace MPCC shadow interfaceを追加する。
8. 判断理由、座標契約、warm-start、target provenanceを構造化ログで確認できるようにする。

## Constraints

- `/control/command/control_cmd`を含むROS 2インターフェースを変更しない。
- Domain、launch、評価成果物、result JSONの契約を変更しない。
- EmergencyBrake、NaN/odometry fail-safe、Recovery、hard clampを弱めない。
- 現行のMPC／OvertakeLine／DynamicEscape authorityは本作業では無効化しない。
- `output/`、rosbag、ユーザー変更中の`aichallenge/result-summary.json`を編集しない。
- 2025 AWSIM由来の競技向け暫定実装として扱う。

## Definition of Done

- 非等間隔horizonでstage waypoint、transition distance、cumulative distanceの対応を単体テストする。
- side別solver contextが連続評価でwarm-start状態を保持し、context変更でresetされる。
- 同一IDでもtarget観測が許容範囲を越えて移動したasync certificateを棄却する。
- Left / Right / Hold / Returnのshadow候補を同じschemaでログ出力できる。
- 対象packageがbuildし、関連単体テストが通る。
