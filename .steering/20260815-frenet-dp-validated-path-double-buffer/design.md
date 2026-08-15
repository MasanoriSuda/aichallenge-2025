# Design

## 原因

現行 rolling refresh は候補の planner-level feasibility を確認した直後に、実行中ベクタへ
`swap` し、その場で旧 runtime validation timestamp を無効化する。その新経路が同周期以降の
live horizon validation を獲得できない場合、旧経路へ戻れず FollowPrepare の単一横位置 fallback
へ落ちる。

## 方針

### 1. Candidate validation before promotion

Behavior 出力に保持されている新しい same-target/same-side candidate を pending buffer とみなす。
現在の実行中ベクタは active buffer として維持し、candidate を次の順で検証する。

1. DP execution reference として全 control horizon を被覆できる。
2. 現在姿勢・速度から wall clearance と lateral acceleration が成立する。
3. locked target が連続し、position jump/course progress reject がない。
4. 現在車体が非重複、または recoverable side contact である。
5. wall contact、emergency、solver recovery、forbidden waypoint がない。

すべて成立したときだけ active buffer を一括置換し、同時に runtime validation timestamp を
更新する。棄却時は active buffer と lease を一切変更しない。

### 2. Candidate attempt throttle

棄却候補を 40 Hz で再検証しないよう、最後の候補検証時刻を active refresh 時刻とは別に持つ。
rolling refresh interval は「最後の採用」ではなく「最後の候補検証」にも適用する。

### 3. Dynamic wait continuity

直前周期に `continuous_dp=1` の physically validated prefix が前進出力を所有し、active DP と
runtime lease が生きている場合、それを committed execution として扱う。短い reselect
time/distance limit は Mission を破棄せず、hard fault・rear-clear・Mission total budget の判断へ
委ねる。

### 4. Lease

実測 revalidation gap が 0.21--0.23 s だったため、runtime validation lease を 0.20 s から
0.30 s にする。これは active/pending 分離後だけ適用し、未検証候補を延命する用途には使わない。

## 安全性

- 新候補の昇格条件は planner feasibility より厳しくする。
- 旧経路の継続も既存 authority resolver の target/wall/emergency/solver hard guard を通る。
- pause retention は直前に検証済み prefix が出力を所有した場合だけ許可する。
- Mission total budget と rear-clear/Return は変更しない。
