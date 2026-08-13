# Design

## 1. 実行速度と完遂予測のcoupling

現在のruntime rear-clear rolloutは、Passでfront capが解除されSafeSeparationが
full-speed forward escapeを選べる場合でも、Missionのnominal closing speed
（最大2.0 m/s）を使う。このため実際の前進能力より悲観的な完遂距離になる。

純粋関数 `resolve_pass_completion_rollout_speed` を追加し、以下を全て満たす場合だけ
rolloutのclosing speedを `v_max - target_speed` まで広げる。

- Pass中
- full-speed forward escape設定が有効
- front cap解除済み
- 現在車体が分離済み
- target予測が有効で、予測sweepも分離済み
- corridor blockとhard faultがない

実際の速度上限はrollout内のコース速度cap、加速度上限、`v_max`が引き続き拘束する。

## 2. Fail-closed境界

次の場合はnominal closing speedへ戻す。

- wall contact / margin fault / wall sample unavailable
- emergency front risk
- solver recovery
- target pass-side intrusion
- execution corridor blocked
- prediction invalid / predicted sweep overlap

したがって安全余裕を縮める変更ではなく、既に安全な横経路で使用する縦速度モデルを
実行時ポリシーへ一致させる変更である。

## 3. ログ

周期 `OvertakeLine debug` に以下を追加する。

- runtime rollout closing speed
- full-speed coupling適用有無
- runtime predicted rear-clear距離・時間

これにより、次回試走で局所上限中断が本当に完遂予測の不整合から減ったか確認する。

## 変更範囲

- `v2x_overtake_core.hpp/.cpp`: typed policy
- `mpc_controller_cpp.cpp`: runtime rolloutへの統合とログ
- `config.yaml`, `config_for_cloud.yaml`: feature flag
- `test_v2x_overtake_core.cpp`: policy境界テスト

