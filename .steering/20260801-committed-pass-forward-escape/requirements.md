# Requirements

## Goal

minimum-motion Passで並走へ到達した車両が、V2X横速度予測の一時的な重複判定で
前車速度へ再拘束されず、現在の車体非重複を保ったまま前方へ抜け切れるようにする。

## Evidence

`output/20260801-165143/d1/autoware.log`ではfront-capが26回再適用され、適用時間は
合計5.799秒、最長2.250秒だった。最長区間ではego速度が4.06 m/sから1.55 m/sへ
低下し、その後にwall RecoveryとSafetyBrakeへ移行した。

## Requirements

- minimum-motion Passだけを対象とする。
- targetが車体縦方向の並走範囲または後方にあり、現在車体が非重複なら、予測重複
  だけでfront-capを再適用しない。
- targetがまだ明確に前方の場合、予測重複が設定時間連続した場合だけ再適用する。
- 初回解除では一時猶予を使わず、予測非重複または並走escape成立を要求する。
- 現在車体重複、V2X位置ジャンプ、実壁接触、target喪失は従来どおり解除を拒否する。
- SafetyBrake、front-risk、壁・横加速度Recoveryは無効化しない。
- Start Grid、inter-vehicle corridor、非minimum-motion Passは変更しない。
- ユーザーのD2 trajectory設定とresult成果物を変更しない。

## Out of scope

- gap幅、壁余裕、横加速度上限の変更
- Recovery FSMの変更
- Pass側選択の変更
- 加速度上限の変更
