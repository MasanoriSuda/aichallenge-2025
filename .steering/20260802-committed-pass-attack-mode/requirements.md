# Requirements

## 目的

競技シミュレーションの通常 Overtake で、検証済みの minimum-motion corridor に入り、
現在の車体 footprint が対象車と非重複になった後に、将来予測の重複だけを理由として
前車速度 cap と SafetyBrake を再適用しない。並走から前後関係を反転するまで前進を継続する。

## 必須条件

- 新規追い越しの front cap 初回解除条件は緩和しない。
- 攻撃モードは Pass、固定 corridor、既に解除済みの front cap、現在車体非重複、target 継続観測時だけ有効にする。
- 現在車体の重複、target position jump／消失、壁接触、実行経路不成立、solver 異常による既存保護は残す。
- body-clear deadline は候補の hard reject ではなく候補選択の優先順位として用いる。
- 設定で無効化可能にし、既定値は false、競技用 config でのみ true にする。
- ROS 2 topic、message、service、提出インターフェースは変更しない。

## 対象外

- 加速度上限 1.0 m/s^2 の変更
- Recovery／Reverse の再設計
- 実車向け設定への適用
- AWSIM 実走による効果確認

