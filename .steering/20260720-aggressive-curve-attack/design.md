# Design

## hard curve新規進入

既存の内側／外側curve resolverへ`hard_entry_enabled`を追加する。
`hard_entry_allowed`は未追い越し、hard curve、対象側gap成立時のみtrueにする。
soft curve進入とhard curve継続は従来フィールドのまま残し、設定省略時の挙動を変えない。

hard entryはcurveに関するstart/completion guardだけを例外化する。明示WP禁止、cooldown、
EmergencyBrake、target検証、gap plannerの壁・車体境界は例外化しない。

## active gap-loss hold

OvertakeLine状態へ最後に有効gapを確認した時刻を保持する。locked sideを評価中に、次の
一時的な拒否だけが発生した場合は最大0.5秒間gapを有効扱いにする。

- gap width不足
- gap time不足
- reachable gapの一時欠落

front distance、lateral acceleration、反対側gap、target position jump、明示WP禁止、
cooldown、EmergencyBrakeはholdしない。有効gapを再確認した周期だけ時刻を更新し、hold自身で
期限を延長しない。

## 進入と継続のgap条件

入口は連続2点と0.5秒、継続は連続1点と0.3秒を使う。prepare distanceは従来どおり
新規進入だけに適用する。

## dev3攻め側設定

- entry front distance 4.0 m、active continuation 1.8 m
- prepare distance 3.0 m、close-follow有効
- guard/line lateral acceleration 6.0 m/s^2
- ShiftOut 4.0 m、Return 6.0 m
- ShiftOut closing 2.0 m/s、Pass未ラッチ1.0 m/s
- rear-clear 2.0 m、confirm 0.10 s、curve cooldown 0.30 s
- hard curve内の内／外新規進入を有効化
- wall clearance 0.1 mとinflated obstacle gap 0.2 mは維持

本設定は2025由来AWSIM dev3比較用で、実車設定には連動させない。
