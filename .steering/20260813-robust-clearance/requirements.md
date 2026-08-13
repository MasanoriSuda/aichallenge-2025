# Requirements

## 目的

追い越し経路が車両・壁の物理境界ぎりぎりを使用し、V2X揺れ、自己位置・地図差、
操舵追従誤差を吸収できない問題を抑える。

## 要求

- 車両間は単なる非重複ではなく、速度・曲率に応じた表面余裕を持つ。
- 壁の物理 hard guard とは別に、速度・曲率に応じた計画用余裕を持つ。
- entry、Pass再計画、front-cap latch、safe trajectory prefixで同じ余裕を使う。
- 狭い区間で余裕を確保できない場合はMissionを成立させず、接触前に再評価する。
- 実接触後のContactContinuationと壁hard guardは従来条件を維持する。
- ROS 2 topic/service/messageと評価インターフェースを変更しない。

## Definition of Done

- 速度・曲率からロバストな車両／壁クリアランスを算出するpure policyがある。
- complete Missionの採用・再構築にロバスト余裕が適用される。
- safe trajectory prefixはロバストな現在／予測車体分離を要求する。
- 物理wall contact判定とContactContinuationは攻撃化・保守化しない。
- unit testと`make autoware-build`が成功する。
