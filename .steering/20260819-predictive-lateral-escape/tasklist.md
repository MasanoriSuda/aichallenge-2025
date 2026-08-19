# Tasklist

- [x] Latest run の Pass exit と runtime wall escape rejection を照合する。
- [x] 現行の target / wall clearance fallback の非対称性を特定する。
- [x] Runtime wall goal resolver を二段階 wall clearance 対応へ整理する。
- [x] Controller から robust / hard wall interval を明示して渡す。
- [x] warning lookahead と bounded correction range を調整する。
- [x] README と設定コメントを更新する。
- [x] positive / fail-closed unit tests を追加する。
- [x] package build と focused test を実行する。
- [x] 意図したファイルだけをコミットする。

## Definition of Done

- ロバスト壁余裕だけが原因の候補棄却時に、hard wall clearance を満たす
  同側横逃げを再評価できる。
- hard wall fault、車体重複、preflight 不成立を上書きしない。
- ログから target / wall の各 clearance mode を判別できる。
- 既存のユーザー変更 `aichallenge/result-summary.json` をコミットしない。

## Validation

- `docker compose run -T --rm --no-deps autoware-build`: 25 packages
  succeeded.
- `colcon test --packages-select multi_purpose_mpc_ros --ctest-args -R
  test_v2x_overtake_core --output-on-failure`: 1334 tests, 0 failures.
- `colcon test-result --verbose` reported only the pre-existing stale
  `joycon_contract_guard/package.xml` result-path warning; selected tests had
  0 errors and 0 failures.
- `git diff --check`: passed.
