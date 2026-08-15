# Design

## 方針

target-bound failureを単なる「現在prefixの保持」ではなく、短い再計画窓として
扱う。処理順は次のとおりとする。

1. 現在の物理的に成立するsame-side prefixを保持する。
2. opponent/MPCC-lite評価タイマを即時再armし、次周期に左右を再評価する。
3. fresh same-side候補があればSafeSeparationを維持したまま置換する。
4. no-return前に限り、fresh alternate候補をtarget-bound rescueとして置換する。
5. 解が得られず、予測sweep不成立のままtargetへ接近した場合は、横側を保持し、
   `target speed + bounded closing speed`へ縦速度だけを制限する。
6. hold budgetまたはhard guardに達した場合は既存Recoveryへ渡す。

## 近距離guard

初期値は次とする。

- guard開始中心間前後距離: 1.5 m
- guard中最大closing speed: 0.2 m/s

これはFollowへ戻る処理ではない。Passとside lockを保持し、fresh trajectoryが
成立すれば直ちに置換して前進を再開する。target速度未満への意図的な後退は
行わない。

## 安全境界

- actual wall contact/margin violationはhard faultのまま。
- current body overlapは既存ContactContinuation条件を満たす場合だけ継続する。
- cross-side置換はno-return前かつ既存のfull-path preflight成立時だけ許可する。
- target-bound failure以外のSafeSeparationにはtactical rearmを波及させない。
