# Dynamic evidence design

`make dev2`を用い、通常Followをlegacy authorityのまま走らせる。同一周期で生成したcanonical
shadowはtelemetryだけに記録されるため、現行走行性能へ制御入力として影響しない。

AWSIMのcwd出力がユーザー所有の`aichallenge/result-summary.json`を変更しないよう、一時Compose
overlayでsimulatorのworking directoryだけを専用output run directoryへ変更する。

判定順は次のとおり。

1. eligible / typed Follow contract
2. solve / normalized primal
3. effective physical gap
4. actuation proposal
5. swept physical wall certificate
6. canonical plan / cursor / candidate
7. fresh authority / actuation identity / command

最初に比率が落ちる境界を次Sliceのroot cause候補とする。
