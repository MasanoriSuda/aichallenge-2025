# Design

## 現行課題

停止・低速車は広いcourse corridorで検出されると、通常の左右Mission生成より前に専用local pathを評価していた。専用経路が不成立の場合は、その周期で`Follow`を返すため、全V2X footprintを扱える通常GapPlannerへ到達しなかった。

## 方針

`resolve_dynamic_obstacle_cruise_authority()`で新規遭遇の所有権を一箇所に集約する。

1. 同一車両の連続V2X観測で停止・低速候補を確認する。
2. course corridor上の候補を通常front tactical targetへ昇格する。
3. 通常GapPlannerが全active V2X車両を時系列制約として左右候補を評価する。
4. complete/progressive Missionが成立すれば通常OvertakeLineへ渡す。
5. 成立しなければ既存Follow/SafetyBrakeへ戻す。

広いfuture-course targetへの昇格は初期値`15 m`以内かつ`2.0 m/s`以下とする。これにより停止車だけでなく、接触後に約5 km/hで動く車両も通常の動的障害物候補へ早期に入る。通常front/sideに分類されるそれ以上の速度の全車両も、従来どおりGapPlannerの時系列footprint制約へ含まれる。

`Overtake`というFSMラベルは当面、動的回避軌道の実行権を示すsupervisor互換ラベルとして残す。横経路の候補生成は停止車専用ではなく通常系が所有する。

## ロールバック

`v2x_dynamic_obstacle_cruise_authority_enabled: false`で従来のlow-speed専用分岐へ戻せる。

## 効果確認

- ログ理由に`all-V2X dynamic-obstacle cruise`が出ること。
- 同じ遭遇で`low-speed gap unavailable`が先行しないこと。
- `Idle -> ShiftOut/Pass`へ入り、停止車の再発進まで待たずに回避開始すること。
- `SafetyBrake`、壁接触、solver failureが増えていないこと。
