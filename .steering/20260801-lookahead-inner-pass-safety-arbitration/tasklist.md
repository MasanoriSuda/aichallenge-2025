# Tasklist

- [x] 最新runと現行ロジックを照合する
- [x] 次カーブ内側の先読み設定と判定を追加する
- [x] minimum-motion候補へ開放距離・幅を追加する
- [x] 十分に開いた内側を選択する純粋関数テストを追加する
- [x] committed corridorのfront-danger抑制判定を追加する
- [x] 不確実・重複・別車両のfail-closedテストを追加する
- [x] 診断ログを追加する
- [x] ビルドとテストを実行する
- [x] 実走確認項目を整理する

## Dynamic verification

- `lookahead_inner`と`inner_pref=1`が、次カーブ前の直線で期待側を示すこと
- validated ShiftOut / Passで`danger_suppress=1`となり、同じtargetへのSafetyBrakeが減ること
- 予測sweep重複時は`danger_suppress=0`のままであること
- `Pass -> Return`成功数が増え、壁Recoveryと接触が増えないこと
