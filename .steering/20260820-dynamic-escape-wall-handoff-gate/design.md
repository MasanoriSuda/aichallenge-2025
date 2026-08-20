# Design

## 欠陥

現行は `solver recovered` を通常MPCへの復帰条件としている。ところが solver が
解けたことと、公開予定の `current_prediction` が車体footprint込みで壁から安全な
ことは別条件である。DynamicEscape用のcandidate/preflight壁検査を通過していても、
復帰先のracing-line解はその契約を継承していない。

## Wall handoff admission gate

`overtake_execution_orchestrator` に状態付きの純粋ゲートを追加する。

入力:

- bounded continuation終了イベント
- 現在footprintの有効性・接触状態
- 最終採用予測のavailability、validity、contact、out-of-map
- 予測の最小壁距離と要求壁余裕
- 必要な連続有効周期数

出力:

- `hold_control`: 通常MPCへの切替を保留するか
- `released`: 物理再認定が完了したか
- `reason`: unavailable / invalid / contact / out-of-map / clearance / requalifying
- 有効連続周期数とhold周期数

ゲートは危険な解を経過時間だけで許可しない。連続2周期の物理的に有効な予測で
解除する。通常走行ではinactiveであり追加制御を行わない。

## 最終指令

保留中は、直前に公開した横指令を制御authorityとして維持する。MPC内部の
steering/control historyも公開指令へ同期し、拒否した解を次周期のwarm startへ
混入させない。縦方向は加速禁止とし、現在速度・直前指令を上限に `a_min` で
段階的に減速する。

現在footprintがすでに不安全なら、ゲートは解除せず停止要求を出す。後段の既存
Recovery arbitrationが成立した場合はRecoveryを優先し、ゲートをresetする。

## ログ

状態変化時に次を一行で出す。

```text
DynamicEscape wall handoff admission: decision=..., event=entered|blocked|released,
reason=predicted-wall-contact, valid=0/2, holds=1,
current=valid/clear/contact, prediction=available/valid/samples,
path_wall=region/distance/index/path_distance/contact/out,
command=held_speed/held_steering
```

既存の `DynamicEscape wall handoff` は接触までの観測証拠として残し、新ログは
「なぜ通常MPC指令を採用・拒否したか」を記録する。

## 非対象

- solver重み・反復数の調整
- DynamicEscape候補生成の変更
- Recoveryアルゴリズムの変更
- 通常走行の全予測に対する新しい常時wall guard
