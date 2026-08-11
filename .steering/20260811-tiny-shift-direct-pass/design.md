# Design

## 方針

### 1. DirectPass候補分類

候補生成時にpure policyでDirectPass資格を決める。

- legacy base-line: goalが基準線上かつ現在位置がcorridor内
- tiny-shift: minimum-motion有効、現在位置がcorridor内、横移動が設定値以下

どちらも従来のentry preflight、kinematic rollout、rear-clear、full Mission、Return検証を
通過した候補だけが実行候補になる。tiny-shiftは安全条件を緩和するものではなく、
検証済み経路の開始stageだけをShiftOutからPassへ変更する。

### 2. Mission距離の整合

DirectPass候補はPass originを現在位置としてrear-clear距離を評価する。固定横目標との
最大0.20 mの差はPass内のsame-side lateral trackingで解消する。ShiftOut距離をPassの
前進予算から重複控除しない。

### 3. Entry stage

`resolve_overtake_entry_stage()`へtiny-shift入力と専用reasonを追加する。優先順位は、

1. base-line DirectPass
2. tiny-shift DirectPass
3. paused same-side DirectPass
4. Safety pause resume
5. normal ShiftOut

とする。pause resumeの分類条件自体は変更しない。

## 非対象

- 0.20 mを超えるShiftOutの速度・加速度変更
- gap、wall margin、collision判定の緩和
- SafetyBrake、Recovery、Reverseロジックの変更
- rearward completion条件の緩和

## 動的確認

- 対象ケースで`Idle -> Pass`、reason=`validated tiny-shift corridor already clear`となる。
- `lateral_shift<=0.20 m`のMissionがShiftOutへ長期滞在しない。
- Pass後もfront-cap releaseと前進所有権を維持し、SafetyBrake/Reverseへ連鎖しない。
- 0.20 m超のMissionは従来どおりShiftOutする。
