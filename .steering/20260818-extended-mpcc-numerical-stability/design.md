# Design

## 1. Relative progress state

拡張QP内では次を用いる。

```text
theta_local = theta_absolute - progress_origin
```

状態上下限、参照値、初期値を同じ原点へ変換する。拡張解を既存の3状態表現へ戻す
ときだけ原点を加算する。warm-startは前周期と現周期の原点差を全theta stateへ加算する。

## 2. Extended-only cost scaling

既存3状態MPCCの重みは変えず、拡張モデルだけに以下を導入する。

- legacy lateral/heading weightへ共通scaleを適用
- lag weightを拡張モデル専用に分離
- thetaへ参照追従二次項と小さい前進報酬を設定

これにより、`1e-6 * theta^2 - 2000..5000 * theta` と最大150万級の追従重みが
同居する条件の悪いQPを避ける。

## 3. Failure circuit breaker

拡張solverが失敗または解変換に失敗した場合、`cooldown_sec`だけ拡張solverを停止する。
停止中は既存3状態MPCCを直接解き、拡張QPを先に試さない。cooldown後のprobeが成功すれば
即時復帰し、失敗すれば再びcooldownへ戻る。

## 4. Telemetry

既存の全OSQP集約ログは互換維持する。追加の `Extended MPCC runtime` は1 Hzで以下を集約する。

- eligible cycles
- solve attempts / successes / failures
- circuit skips
- build / conversion rejects
- legacy fallbacks
- solve time avg/max、iteration avg/max

周期ごとのログは追加しない。

## 影響範囲

- `mpcc_progress.hpp/.cpp`: progress変換、warm-start再基準化、circuit breaker
- `mpc_controller_cpp.cpp`: QP組立、fallback制御、telemetry
- `config.yaml`: 拡張モデル専用の数値安定化パラメータ
- `test_mpcc_progress.cpp`: 純粋関数・circuit breakerテスト
