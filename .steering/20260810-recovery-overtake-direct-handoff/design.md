# Design

## 原因

`validated_forward_overtake_escape_available` は新規の coordinated-stop Recovery を
抑止するだけだった。開始済み episode は `recovery_coordinated_stop_episode_` により
保持され、前進 Mission が再成立しても LowSpeedRejoin 完了まで Recovery が制御を
所有していた。

## 方針

1. pure core に `resolve_forward_overtake_handoff()` を追加する。
2. ハンドオフ対象を simulation の coordinated-stop episode に限定する。
3. pre-arm は完全な Mission と hard guard を短時間連続確認する。commit 済み Mission
   は同一 target、現在車体非重複、実行 corridor 有効を要求する。
4. Recovery の静的 footprint、前進 path、V2X 完全性も再確認する。
5. Reverse/Neutral 中は次の順で段階移行する。
   - 後退中: HoldStop
   - 停止後: RequestDrive
   - fresh Drive report かつ非後退: Recovery を解除
6. 解除時は V2X / Overtake Mission を reset せず、Recovery 内部状態だけを終了する。
   再発防止用の bounded forward rearm guard は有効化する。

## 失敗時

候補が消えた場合、または壁・solver・collision worsening を検出した場合はハンドオフを
取り消し、既存 Recovery に制御を戻す。Drive report 未確認では通常制御へ返さない。

## 実走ログ

- `Stuck recovery forward Overtake handoff candidate armed`
- `Stuck recovery stopping for forward Overtake handoff`
- `Stuck recovery gear requested: gear=Drive`
- `Stuck recovery handed off directly to validated forward Overtake`

