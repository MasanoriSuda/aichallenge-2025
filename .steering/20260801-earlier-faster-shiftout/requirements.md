# Earlier/Faster ShiftOut A/B 要件

## 目的

低速車に接近し過ぎてから横移動を始め、並走成立までに時間を消費している仮説を、
設定だけのA/Bで確認する。

## 基準走行

`output/20260801-002034/d1/autoware.log`

- 新規`Idle -> ShiftOut`: 3回
- 全`ShiftOut -> Pass`: 10回
- `Pass -> Return`: 0回
- `ShiftOut/Pass -> Recovery`: 12回
- ShiftOut開始からPassまで: おおむね1.8～3.2秒

## 変更条件

- 通常の前方認識・Follow準備範囲を8 mへ広げる。
- 新規ShiftOutは前方距離6 m以上、かつ6 m以上先に連続した候補経路がある場合に限定する。
- ShiftOut中のadaptive closing speedを0.8～2.0 m/sとする。
- Pass未latch時の0.5 m/s制限、壁判定、SafetyBrake、Pass継続処理は変更しない。

## Definition of Done

- 変更がparam yamlだけに閉じている。
- `make autoware-build`が成功する。
- 次回試走で新規ShiftOut開始距離、ShiftOut所要時間、Pass完遂数を比較できる。

