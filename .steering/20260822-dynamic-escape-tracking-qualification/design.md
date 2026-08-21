# Design

## 問題

最新走行では primary 候補が GapPlanner、reachable bridge、static wall preflight を通過して lateral authority を得た直後、同じ周期の実追従 QP が warm/cold の両方で失敗した。幾何判定と追従 QP の可行性が一致せず、未実行の候補失敗が減速 fallback として車両へ現れていた。

## 方針

最初の実追従 QP を候補の資格確認として扱う。

1. 幾何条件を通過した候補は `qualification-pending` とする。
2. QP が成功した時点で target/side scoped の資格を記録する。
3. QP が失敗した場合は同 target/side を backoff へ入れ、候補を非 active に戻す。
4. 失敗候補の解は存在しないため publish せず、直前の有限な通常走行速度と操舵を1周期保持する。
5. 直前指令が利用不能なら従来の減速 fallback を使う。

成功時の solve 回数は増えない。失敗時も同一周期の二重 solve はせず、次周期に backoff 済みの通常経路を解く。

## ログ

- planning outcome: `qualification-pending-primary|alternate`
- tracking outcome: `qualified|qualification-rejected|failed|recovered`
- qualification rejection には branch、solver理由、hold可否、保持速度・操舵を出す。
- `failed` は既に資格確認済みの branch が実行中に失敗した場合だけに限定する。

これにより「候補生成失敗」「採用前QP失敗」「実行中QP失敗」をログだけで分離できる。

## 対象外

- OSQP 制約行ごとの残差診断
- topology fingerprint による warm-start reset
- Recovery heading/rejoin gate
- パラメータ調整
