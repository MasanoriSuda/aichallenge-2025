# Requirements

## 目的

ロバストクリアランス導入後、壁warningを検知してもfreshな同側Missionが得られない場合に
凍結済みの壁側goalを継続し、hard wall guardまたは横加速度Recoveryへ到達する問題を抑える。
また、Passで一度速度解放した後、実寸車体は非重複なのにロバスト推奨余裕の一時的な割れだけで
front-capが再適用される問題を抑える。

## 要求

- runtime wall warning後は、fresh同側Missionを最優先する。
- fresh候補が一定時間得られなければ、同じpass sideと名目車間を維持した中央寄りgoalを
  完全preflightしてから原子的に置換する。
- 中央寄りgoalも成立しない場合、対象が十分前方かつReturn corridorが空いている場合だけ
  RecoveryではなくReturnへ移る。
- actual wall contact、hard wall margin、実寸車体重複、予測不能は緩和しない。
- front-capの初回解除は従来どおりロバスト車体sweepを要求する。
- 解除後のPass保持は、実寸車体と実寸予測sweepが非重複ならロバスト推奨余裕の瞬間的な割れを許す。
- ROS 2 topic/service/messageと評価インターフェースを変更しない。

## Definition of Done

- wall warningのfallback actionをpure policyでテストできる。
- 中央寄り縮退goalは移動量、同側、壁範囲、名目車間を検証してから採用される。
- front-cap初回取得と取得後保持の安全条件が分離される。
- core unit testと`make autoware-build`が成功する。
