# Validation

## Static

- `make autoware-build`: success。
- `test_persistent_osqp`: 9/9 passed（A/B実装時）。
- `test_race_mpcc_foundation`: 20/20 passed。

## Dynamic A/B

- Run: `output/20260823-142145`
- 多くの1秒windowでaccepted率0--58%。
- `stage=constraint_check`で元単位のmax violation 0.02--0.03が頻発。
- `stage=row_certificate`のworst rowは270が多く、normalized violationは最大15程度。
- solve時間自体は概ね1--5 msへ短縮したが、粗い解を早く返した結果であり採用不可。

## Decision

- scaled termination + polish: rejected。
- row-wise tolerance relaxation: prohibited、未実施。
- production接続: 未実施。
- A/Bコード: revert済み。
- 次候補: extended QPの単位正規化とcurvature-rate rowの可行性監査。
