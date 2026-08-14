# Requirements

## 背景

`output/20260814-214306` では、OvertakeLineは5回ともPassへ到達したが、正常な
`Pass -> Return -> Idle` は0回だった。代表例では現在車体が一度非重複になった時点で
receding-horizonの相手制約を解除し、targetがまだ前方にいるにもかかわらず横経路が
target側へ戻った。その後、再重複または壁側不成立からRecoveryへ移行した。

## 目的

- Pass中の相手制約をbody-clearでは解除せず、rear-clear確認まで保持する。
- rear-clear後にReturnのlive preflightが不成立でも、現在側を物理的に保持できる間は
  Pass内で再計画を継続する。
- Return不成立を直ちにRecoveryへ変換しない。
- wall接触、未知領域、EmergencyBrakeなどのhard faultは従来どおり優先する。

## 制約

- ROS 2 topic、message、service、launch entry、評価結果schemaは変更しない。
- 車体・壁の物理制約を緩和しない。
- rear-clear前のtarget footprint制約を省略しない。
- Return延期中も同じsideとMission generationを維持する。

## Definition of Done

- body-clearのみでは相手制約を解除しない単体テストを追加する。
- rear-clear確認後だけ相手制約を解除できることをテストする。
- Return preflight失敗時に、物理的に成立するcurrent-side holdをPass内で採用する。
- current-side holdも不成立なら従来のhard-fault/Recovery経路を維持する。
- 対象packageのテストとビルドが成功する。
