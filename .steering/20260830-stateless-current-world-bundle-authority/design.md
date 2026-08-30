# Design

## Authority invariant

historical artifactは次だけを提供する。

- target/homotopy identity
- immutable control sequence
- wall/course snapshot fingerprint
- phase intent

実行authorityは毎周期、現在世界から作る次のBundleが所有する。

- measured-to-control connector
- current control-origin state
- exact nonlinear continuation
- wall/peer certificate
- terminal successor viability
- current serialized actuation

## Progress semantics

`lift_progress()`は周回を跨ぐ現在位置をartifactと同じ連続座標へ載せるために使う。
artifactとの差はlifecycle driftのdiagnosticであり、current-world Bundleの物理証明より強い
拒否条件ではない。

差が従来toleranceを超えた場合は`progress_rebased=true`とし、proofをstatelessとして扱う。
このproofはcommand publicationできるが、未変更source planをexecuted historyとしては扱わない。

## Lifecycle

1. source cursorからimmutable control suffixを選ぶ。
2. 現在pose/speed/serialized steeringをcontrol originに固定する。
3. current-world continuation、wall、peer、terminal successorを証明する。
4. 証明成功時、progress rebaseの有無をproofへ記録する。
5. rebaseありならpublished Bundle sourceとして現在のcontrol origin/cursorをledgerへ記録する。
6. 次周期はそのpublication joinを起点に再証明する。

## Deleted legacy path

- `Reason::ProgressLiftRejected`
- `evaluate_impl(..., enforce_progress_continuity)`二重評価
- observation-only stateless rebase結果
- production前段のhistorical progress hard reject

## Non-goals

- candidate/homotopy生成の変更
- SQP回数やOSQP設定の変更
- 速度・操舵・クリアランスparameter tuning
- cursor exhaustionやsingle-SQP limitationの同時修正
