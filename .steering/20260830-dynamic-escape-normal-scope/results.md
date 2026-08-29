# Results: DynamicEscape normal-scope integration

## Root-cause conclusion

`bfaf7333`でpre-Mission DynamicEscapeのcanonical identityはCruiseへ降格したが、
`mpcc_progress::resolve_activation()`には古いDynamicEscape execution activationが残っていた。
そのため正常系Track/Cruise populationが`live-progress-already-active`で拒否される一方、
Overtake Mission identityがないのでexecution populationも作れず、normal authorityが空になっていた。

本Sliceはこの二重ownerの残片を削除した。DynamicEscapeはTrack/Cruise normal populationだけが所有し、
ShiftOut／Pass／Return execution scopeはcoherent canonical Overtake identityだけが起動する。
wall、timed obstacle、terminal successor、Store、publisherのcurrent-world証明は変更していない。

## Static validation

- source-contract: 78 passed
- package CTest: 54/54 targets、2125 tests、0 failures
- `make autoware-build`: 25 packages passed
- `git diff --check`: passed
- config、solver tolerance、weight、clearance、lease、grace、timeout、retry、fallback変更: なし

## Dynamic validation

### Baseline

- `output/20260830-005711`
- DynamicEscape中に`track=0/follow=0/execution=0/rejoin=0`となるmissing-scope decisionを53件観測
- 最初の代表事例はdecision 2725

### 除外run

- `output/20260830-011559`
- AWSIMがSpawnedからGroundedへ進まず、MPC control cycleが開始しなかった
- コード評価には使用しない

### Accepted candidate

- `output/20260830-011957`
- `action=dynamic-escape`: 13 decision
- 旧missing-scope signature: 0
- DynamicEscape中Emergency authority: 0
- certified normal solution publication: 13/13
- 7-state authority: 13/13
- 実行contractがCruiseだった7件の内訳:
  - fresh certified candidate: 2
  - current-world Bundle: 3
  - executed retained: 2
- 正式なMission遷移も維持:
  - `Idle -> ShiftOut`: 5
  - `ShiftOut -> Pass`: 1

代表decision 1790は実速5.07 m/sから5.14 m/s、1798は5.21 m/sから5.28 m/s、
1804は5.28 m/sから5.34 m/sをnormal authorityでcommandした。従来のmissing-scope Emergencyは
名前を変えただけではなく、同じDynamicEscapeでnormal producerが実際にpublishした。

## Acceptance boundary and residual risks

この結果が閉じるのはDynamicEscapeのexecution activation残存によるscope欠落だけであり、
レース品質全体やPass完遂の合格ではない。

- 13 decisionのうち6件はtrace上のcanonical Cruiseに対し、実際のexecution contractが
  retained ShiftOutまたはFollowだった。certified artifactではあるが、traceとpublication contractの
  semantic handoffを次Sliceで監査する必要がある。
- ordinary Cruise artifact unavailableからEmergencyになる別familyが残る。
- ShiftOut dynamic-obstacle max-iterations、progress-lift rejection、cursor exhaustionが残る。
- 本runでは`Pass -> Return`を観測していない。

これらを本Sliceの理由でparameter調整または新fallbackへ混在させない。
