# Design

## Detection and timing

- stopped entry thresholdを0.28 m/s、hysteresis releaseを0.35 m/sとする。
- 前方停止車の閾値も0.28 m/sへ揃える。
- stationary/coordinated confirmationを0.25秒、AWSIM settleを0.30秒、stop confirmationを
  0.10秒とする。停止開始からclearance確認までの目標固定待機は約0.65秒である。

## Direction ownership

`recovery_reverse_intent_latched_` はReverseという方向だけを保持する。候補primitiveと操舵値は
従来どおりactuation path到達時にcommitし、それ以前は毎周期のpose/map/V2Xで再評価する。
これによりAWSIM補正への追従と、ReverseからForwardへの意図しない変更防止を両立する。

Reverse-first障害物は、前進要求かつ低速で、次のいずれかを満たす場合とする。

- 停止前方車によるcoordinated stop
- 前方車を伴うrecent collision
- Rear以外の有効なwall/contact分類

Rear wallだけの場合は既存のForward escapeを維持する。

## Explicit fallback

協調停止とsolver reverse-onlyはForwardへfallbackしない。非協調のReverse-first episodeが
`SafeStop -> AggressiveRetry`へ進んだ場合だけdirection latchを解除し、次周期から既存の
Forward candidate比較を許可する。V2X rear blockerによるclearance timeoutは既存どおり停止を
保持するため、このfallback条件には入らない。

