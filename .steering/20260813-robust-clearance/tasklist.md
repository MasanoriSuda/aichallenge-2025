# Tasklist

- [x] 最新ログと現行クリアランス経路を確認する
- [x] ROS 2・評価インターフェースへの非影響を確認する
- [x] robust clearance pure policyとunit testを追加する
- [x] candidate／preflight／実行horizonへ壁計画余裕を適用する
- [x] target goal／front-cap／safe-prefixへ車両余裕を適用する
- [x] config読み込みと起動時ログを追加する
- [x] core unit testを実行する
- [x] `make autoware-build`を実行する
- [x] 差分をレビューする

## Verification

- `make autoware-build`: 25 packages成功
- `ctest -R test_v2x_overtake_core --output-on-failure`: 1/1成功
- `git diff --check`: 成功
- ROS topic/service、評価JSON、提出物構造の変更なし

## 動的確認（ユーザー試走）

- `make dev2`で同一条件を再走する
- `robust_clear=0`のまま速度解放されないことを確認する
- `robust_target`、`robust_wall`と実際の最低車間・壁余裕を照合する
- ContactContinuation回数、壁接触、Pass -> Return完遂率を変更前と比較する
