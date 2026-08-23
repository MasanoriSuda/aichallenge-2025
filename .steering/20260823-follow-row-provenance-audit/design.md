# Follow row provenance audit design

## Data flow

```text
physical A,l,u + raw primal
  -> row-wise residual report
  -> worst-row physical diagnostic
       row/value/projected/lower/upper/violation/tolerance/normalized
  -> extended QP row decoder
       kind/field/stage
  -> Follow shadow decision log
```

診断値はsolver-spaceではなく、既存physical certificateと同じ元の`A,l,u`から作る。
したがってpreconditioning policyの有無に依存しない。

## Extended row layout

`N` horizon、state dimension 5、input dimension 3に対し、row layoutは次とする。

1. dynamics equality: `5*(N+1)` rows
2. state/input box: `5*(N+1)+3*N` rows
3. curvature-rate: `N` rows

各range外、負row、不正Nは`Invalid`として扱い、推測で分類しない。

## Authority boundary

このSliceはmeasurement-onlyである。diagnosticは採否、warm-start、solver reset、commandへ
影響させない。Follow shadowは引き続き`authority=shadow, selected=0`を維持する。
