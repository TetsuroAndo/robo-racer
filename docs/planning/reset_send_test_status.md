# reset_send_test 取り込みステータス

## 目的
- `feat/firmware/reset-send-test` からリセット関連のみを安全に取り込む。
- `rpi/rplidar_sdk` の削除は入れない。
- `platformio.ini` 変更は必要性を見て判断する。

## 状態
- 進行中: 取り込み対象コミットの選定
- 未着手: リセット関連コミットの cherry-pick
- 未着手: `rpi/rplidar_sdk` 削除の回避
- 未着手: `platformio.ini` 変更の確認と反映
- 未着手: 追加/移動されたリセット用ファイルの整合確認

## メモ
- マージではなく cherry-pick で取り込む方針。
