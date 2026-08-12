# Tasklist

- [x] 最新試走ログと現行FSMを照合する
- [x] requirements/designを作成する
- [x] adaptive retry/incident reset距離を2.0 mへ変更する
- [x] rolling再計画後も改善接触追跡を維持する
- [x] rolling接触悪化の連続確認ゲートを追加する
- [x] 単体テストを追加する
- [x] Docker内package build/testを実行する
- [x] 動的確認項目と検証結果を記録する

## Definition of Done

- Rejoin後2.0 mの通常前進で履歴がresetされる。
- rolling再計画境界の連結・非増加接触は`new_contact`にならない。
- rolling Reverse中の1周期だけの接触悪化ではReverseを継続する。
- 0.20秒以上の持続悪化とhard faultは従来どおり停止する。
- build/testが成功する。

## 動的確認

- `make dev2`
- D1の`incident completed after normal travel`がRejoin後約2 mで出ること
- fresh incidentの`adaptive_retry`が0、`escape_target`が4.0 mになること
- D2で0.4 m境界直後の`runtime_contact=new_contact`が消えること
- `contact worsening pending`後にraw判定が解消した場合、ギアを変えないこと
- 持続悪化では`collision_worsening`が発生し、壁へ押し続けないこと

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 targets）
- `colcon test-result --verbose`: 1048 tests、0 errors、0 failures、0 skipped
  - 別packageの古い`build/joycon_contract_guard/package.xml`欠損警告は出るが、
    今回対象packageのテスト結果とexit codeは正常。
- `make dev2`: 未実施。上記「動的確認」をユーザー試走で確認する。
