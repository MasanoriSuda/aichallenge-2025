# Requirements

## 背景

`output/20260814-072352`では追い越しを1回開始して`ShiftOut -> Pass`まで到達したが、
`Pass -> Return`は0回だった。失敗後にfreshな完全Missionを再選択した直後、実測相対速度が
`-0.85 m/s`だったためpre-armへ戻り、その後は前車とほぼ同速のFollowを継続した。

pre-armは`desired_v=約6.1 m/s`を生成していたが、MPC参照への反映が
`min(base trajectory reference, desired_v)`だった。このため通常trajectoryが約4 m/sの
区間ではpre-armが加速要求にならず、相対速度ゲートを自力で満たせない。

## 目的

- 完全Missionまたはbody-clearまで成立したsetupを持つpre-armが、動的hard cap内で
  実際に縦速度参照を引き上げられるようにする。
- 同一targetの一時的なcandidate再検証失敗中も、短いvalidation leaseの間は縦方向の
  pre-armだけを維持する。
- lateral ownershipはfreshなMissionがある周期だけに限定する。

## 制約

- EmergencyBrake、SafetyBrake、wall/contact hard fault、V2X異常を緩和しない。
- lease中は横経路を実行せず、base racing lineを維持する。
- 速度参照は`umax_dyn`、`v_max`および後段のhard velocity limitを超えない。
- topic/service/Domain/提出インターフェースは変更しない。

## 完了条件

- pre-arm参照がbase trajectory速度より高い場合、hard cap内で参照を引き上げる。
- inactive時およびhard capが低い場合の挙動を単体テストする。
- 短時間のsame-target planning missだけpre-arm leaseを許可する単体テストを追加する。
- 対象packageのbuild/testが成功する。
