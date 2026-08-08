# Design

## 1. Forward-completion速度

SafeSeparationの通常分離動作は従来の`target_speed +/- speed_delta`を維持する。
一度forward completionへcommitした場合だけ、設定が有効なら速度参照を`v_max`へ
解放する。これはhard速度指令ではなくMPC参照の上限解除であり、曲率、壁、入力上限、
Emergency、solver guardは引き続き優先される。

## 2. Mission整合SafeSeparation budget

SafeSeparation開始時に以下をfreezeする。

```text
required = max(
  configured local distance,
  mission predicted rear-clear pass distance - current pass distance + margin)

local distance = min(required, absolute pass distance remaining)
local time = min(
  max(configured local time, local distance / current forward speed + margin),
  absolute pass time remaining)
```

Mission予測が無効な場合は従来値へfallbackする。開始後にbudgetを縮めない。

## 3. Contact Continuation

確定overlapを次の条件でrecoverable side contactとして分類する。

- Pass中、front-cap release済み、forward completion latch済み
- locked targetが連続して観測されている
- targetが選択済みPass側の反対に存在し、横離隔がfront-impact閾値以上
- target前後位置がside-by-side window内
- 縦closing速度がhigh-energy接触閾値以下
- 横相対速度が観測できる場合は上限以内
- egoが走行中
- 接触開始直後、または前方進捗がfresh
- 接触継続時間が上限以内

active中はgeneric front dangerを抑制し、確定overlapをboundedなforward completionの
許容幾何として扱う。横目標へPass側の小さなbiasを追加するが、既存wall horizonで
clamp/abortする。

次はContact Continuation対象外とする。

- pass sideと同側にtargetがいる、または横離隔が小さい正面寄り衝突
- 壁接触／壁sample欠損
- target jump／不連続
- solver recovery
- timeoutまたはfresh progressなし

## 4. 観測性

既存のOvertakeLine debugへ次を追加する。

- `forward_full_speed`
- `safe_sep_budget=time/distance`
- `contact_continue`
- `contact_elapsed`
- `contact_progress`
- `contact_bias`

状態開始・終了はedge logだけを出し、周期ログを増やしすぎない。

## 5. 初期設定

- full-speed forward completion: enabled
- Mission budget margin: 1.0 m / 0.5 s
- Pass absolute distance: 40 m
- Contact Continuation: enabled、最大0.8 s
- 最小横離隔: 0.75 m
- 最大縦closing速度: 3.0 m/s
- 最大横相対速度: 0.5 m/s
- 最小ego速度: 0.5 m/s
- contact lateral bias: 0.10 m
