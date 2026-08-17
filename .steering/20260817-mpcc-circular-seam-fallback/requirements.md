# Requirements

## 目的

- 周回境界を含むOvertake Returnでprogress MPCCが連続失敗し、減速fallbackで停止する不具合を解消する。
- progress MPCC固有の前処理不成立を、車両全体の制御失敗へ昇格させない。

## 実走で確認した原因

- `20260817-130843/d1`では、Return中にhorizonが周回境界へ到達した。
- 内部参照経路に長さ0の循環境界stageがあり、progress reference生成が680周期連続で失敗した。
- 各失敗がdeceleration fallbackを発生させ、実速度が6.499 m/sからほぼ0 m/sへ低下した。

## 制約

- 既存のReferencePath、DP corridor、ROS interfaceは変更しない。
- 非有限値や負のstage距離は修復せず、legacy MPCへ縮退する。
- 循環境界の有限な0 m stageだけを局所的な最小正距離へ正規化する。
- 周回進捗のwrapではOSQP warm-startを再利用しない。
- progress MPCC無効時のlegacy MPC挙動を変えない。

## Definition of Done

- 0 m stageを含むprogress horizonが構築できる。
- progress前処理が成立しない周期はlegacy MPC problemを返す。
- 周回進捗wrapでOSQP historyがresetされる。
- 対象packageのbuild/testが成功する。
- ユーザー所有の設定・走行結果をコミットしない。
