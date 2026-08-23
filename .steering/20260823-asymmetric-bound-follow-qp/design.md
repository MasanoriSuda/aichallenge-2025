# Asymmetric-bound Follow QP design (rejected candidate)

## Root cause

従来はtwo-sided rowのcharacteristicを

`max(abs(lower), abs(upper))`

としていた。これは非対称boundの緩い側を常に採用する。実際に小さい側を破った場合、
post-solve certificateのtoleranceよりsolver-space toleranceが大きくなる。

## Evaluated correction

各有限sideについて

`t_lower = eps_abs + eps_rel * abs(lower)`

`t_upper = eps_abs + eps_rel * abs(upper)`

を計算し、row toleranceを

`t_i = min(t_lower, t_upper)`

とする。one-sided rowはその有限side、equality rowは同じ値、unbounded rowはscale 1を使う。
最終scaleは従来どおり`S_i=T/t_i`である。

一つのrow scaleで両sideの異なるtoleranceを完全一致させることはできない。この方針は
厳しい側を一致させ、緩い側を必要以上に厳しくするfail-safeな近似として評価した。

## Rejection reason

既存のmixed-unit回帰試験で、`[0, upper]` のlower sideが常に
`eps_abs + eps_rel * 0`を選ぶため、単位差を吸収するrow scaleが実質的に失われた。
その結果、正規化後もupper boundの大きな逸脱をOSQPがsolvedとして返し、12件中1件が失敗した。

これはparameterの問題ではなく、単一solver rowへ単一scaleしか持てない表現上の制約である。
同じ方式を閾値調整で救済しない。次候補はlower/upperをsolver内部で別々のone-sided rowへ展開し、
physical problemとcertificateは元のtwo-sided rowのまま維持する構造変換とする。

## Authority boundary

policy接続は既存どおりFollow shadowだけ。production、publisher、legacy ownerは変更しない。
候補実装は動的authorityへ接続せず、静的gate失敗後にすべて巻き戻した。
