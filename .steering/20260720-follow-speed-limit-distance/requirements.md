# Requirements

## 目的

dev3シミュレーションで、前方車の早期検出24 mを維持しながら、Followへ入っただけで
遠距離から3 m/sへ固定される挙動を解消する。移動中の前車へ速度を合わせ始める距離は
5 mとし、2〜5 mというチューニング目安の上限側から評価する。

## 要件

- 前方車の共通コース進捗検出は24 mのまま維持する。
- 新規`v2x_follow_speed_limit_distance`でgeneric Follow速度制限の開始距離を指定する。
- 設定省略または0 mでは従来どおり検出距離全域でFollow速度制限を適用する。
- 移動中の前車には`front speed + moving margin`を使い、固定`follow_velocity`で上書きしない。
- 低速・停止前車には距離停止上限と`follow_velocity`の小さい方を使う。
- front risk、curve risk、front decel guard、SafetyBrakeは距離ゲートの外側でも維持する。
- lateral clearanceを確定したactive Passではgeneric Follow capを再適用しない。
- topic、service、message、Domain、評価インターフェースは変更しない。
- 2025 AWSIM由来dev3シミュレーション向け暫定値であり、2026公式値とは扱わない。

## 完了条件

- 5 m境界、移動前車、低速前車、legacy、省略・抑止条件を単体テストする。
- `make autoware-build`が成功する。
- 仕様とA/B設定を`docs/spec/mpc-integration.md`へ記録する。
