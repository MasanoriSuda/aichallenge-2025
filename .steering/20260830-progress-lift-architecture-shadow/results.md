# Results

## Static verification

- `make autoware-build`: 成功。
- `multi_purpose_mpc_ros`: 59/59 CTest成功。
- `colcon test-result --verbose`: 2241 tests、0 errors、0 failures。
- productionの`Reason`と`proof`は変更せず、Bの結果はtelemetryだけへ保存した。
- source-contract testで、Bのproofがpublisherへ渡らないことを固定した。

## Dynamic run

- Run: `output/20260830-152829`
- Mode: `make dev2`

### Episode 1

完全なphase chainを正常完遂した。

```text
Idle -> ShiftOut
ShiftOut -> Pass
Pass -> Return
Return -> Idle
```

### Episode 2

`ShiftOut -> Pass -> Return`までは成立したが、Return実行中のdecision 3157、
sequence 2454でA/Bが分岐した。

```text
A: progress-lift-rejected
control physical progress: 227.409426 m
artifact expected progress: 228.951085 m
delta: -1.541660 m
tolerance: 1.500000 m

B: accepted
complete current-world proof: available
```

BはAと同じcontrols、target/homotopy identity、current worldを使い、現在の
pose/speed/serialized steeringからcontinuationを再構築した。wall、dynamic obstacle、
terminal Stop suffixを省略せず`accepted/proof=1`となった。

Aのreject直後、productionは3.07 m/sから`-3.00 m/s2`のEmergency Stopへ移行した。
Stopがretainedされ、Returnのcurrent-world candidateも後続周期ではcursor exhaustionと
single-SQP rejectへ波及した。その後、左壁接触、AWSIM recovery、Reverseが観測された。

## Classification

`A fails, B succeeds`であるため、今回の根本分類は次である。

> Persistent Mission/artifact lifecycle defect

物理的に走行不可能だったのではない。publish済みartifactのtime/progress clockを
current-world proofより先にauthority prerequisiteとして適用したため、実行可能な
ManeuverBundleを拒否した。

candidate generation defect、single-SQP limitation、clearance parameter不足は、この
first failureの原因ではない。後続のQP不成立と壁接触は、最初のauthority喪失と急制動から
派生した症状である。

## Next slice

- stateless current-world Bundleをretained authorityの正本へする設計を切る。
- target/homotopy、commit/no-return、last published certified artifactだけをpersistentにする。
- path samples、cursor、phase-transition trajectoryはcurrent worldから再構築する。
- current-world full proofが成功した場合にhistorical progress gateでrejectしない。
- production昇格と同じSliceで、旧artifact-clock authority branchを削除する。
- tolerance、lease、grace、fallback、solver設定、clearanceは変更しない。
