# Requirements

## 目的

先行車の横移動によって現在の追い越し側が塞がり、反対側に新しい空間が生まれた場合に、追従または Recovery へ落ちる前に反対側を再評価して追い越し計画を置換できるようにする。

対象は操舵応答の改善ではなく、時々刻々変化する相手位置に対する戦術判断の改善である。

## 現行事象

現行の通常追い越しは、mission admission 時に `OvertakePassPlan` を生成し、`mission_path_frozen` として固定する。

一方、既存の early side replan は `!mission_path_frozen` を要求するため、通常の凍結済み mission では実質的に利用できない。現在側の継続が難しくなった場合、反対側の再評価よりも same-side extension、SafeSeparation、FollowPrepare、Recovery が先に選ばれやすい。

`output/20260806-100528/d1/autoware.log` では以下を確認した。

- `frozen ShiftOut mission continuity`: 18 回
- `committed active pass continuity`: 31 回
- 左右どちらかが `not evaluated`: 多数
- `OvertakeLine early side replan`: 0 回

そのため、映像上は「右側が開いたのに旧左側計画を保持し、追従へ戻る」と見える。

## 必須動作

- 同一 target の追い越し mission 中、no-return point より前では左右の完全経路候補を定期的に shadow 評価する。
- shadow 評価は実行中の操舵目標や凍結計画を直接変更しない。
- 現在側が不成立で反対側が成立する場合、または反対側の物理余裕が設定差以上優位な状態が設定時間継続した場合だけ、反対側計画へ原子的に置換する。
- 置換候補は、現在状態から ShiftOut/横遷移、Pass、rear-clear、Return まで一貫して成立することを確認する。
- 反対側への横断中も、現在および予測 footprint、壁余裕、横加速度、target continuity、別車両 corridor を既存と同等以上に検証する。
- target と横並びになった後、車体重複中、rear-clear 後、Return/Recovery 中は反対側へ切り替えない。
- side replan は 1 mission あたり最大 1 回とし、切替後は新しい側を固定して抜き切る。
- 現在側が継続不能になった場合の優先順位を、反対側置換、same-side SafeSeparation、FollowPrepare/Recovery の順にする。
- side replan によって Pass 全体の絶対時間・距離 budget をリセットしない。
- 現行の SafeSeparation progress extension と hard safety gate を維持する。

## 非目標

- 毎制御周期で実行経路や横目標を連続的に左右へ動かすこと。
- 横並び後に相手を横切ること。
- gap、壁余裕、車体寸法を緩和して候補を成立させること。
- `v2x_prediction_use_course_progress` または `v2x_prediction_use_course_lateral_velocity` を設定変更だけで有効化すること。
- Recovery、stuck recovery、速度・加速度上限を変更すること。
- ROS 2 topic、message、launch、評価インターフェースを変更すること。

## 制約

- 参加者実装 `multi_purpose_mpc_ros` 内へ変更を閉じる。
- frozen mission の一貫性を維持し、置換は完全な新計画が成立した場合だけ行う。
- 瞬間的な V2X 横揺れで左右がチャタリングしないよう、優位差と継続時間のヒステリシスを持たせる。
- 2025 由来の現行 V2X 契約を維持する。

## 完了条件

- 通常の frozen mission 中にも、no-return 前なら両側 shadow 評価が実行される。
- 反対側の一時的な開放では切り替えず、継続した有効候補だけを採用する。
- side replan 後に旧側の goal、corridor、PassPlan が残らない。
- 1 mission 中の左右切替回数が 1 回以下である。
- side-by-side 以降の左右横断が発生しない。
- 反対側が成立する状況で、SafeSeparation または FollowPrepare へ直接落ちない。
- 単体テストと `make autoware-build` が成功する。

