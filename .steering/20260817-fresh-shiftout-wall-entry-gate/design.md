# Design

## Entry gate

新規入口stageを決定した直後、phase遷移とMission freezeより前に次を評価する。

```text
fresh Mission
AND entry stage == ShiftOut
AND (actual wall physical contact OR current wall warning)
```

`current wall warning`は、runtime wall preplan warningのうち将来予測由来ではなく、
現在poseのwarning footprintが壁reserveへ到達して`ttc=0`となったものだけを指す。

該当時はIdleのまま出力を返す。side lockやMission freezeを行わないため、通常経路追従または
stuck recoveryで壁状態が解消した後、最新の左右候補から再度入口を選択できる。

## Ownership boundary

- 将来TTCが正のwarning: gate対象外。既存runtime preplanでsame-side再計画する。
- direct Pass: 横ShiftOutを伴わないためgate対象外。
- FollowPrepareからのpaused Mission resume: 新規Missionではないためgate対象外。
- active ShiftOut/Pass: gate対象外。既存のprefix、hard fault、ContactContinuationを維持する。

この境界により、壁状態から新しい横移動を始める不整合だけを除去し、追い越し全体を
保守化しない。
