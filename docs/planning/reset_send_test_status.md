# reset_send_test 取り込みステータス

## 目的
- `feat/firmware/reset-send-test` からリセット関連のみを安全に取り込む。
- `rpi/rplidar_sdk` の削除は入れない。
- `platformio.ini` 変更は必要性を見て判断する。

## 状態
- 完了: 取り込み対象コミットの選定
- 完了: リセット関連コミットの cherry-pick
- 完了: `rpi/rplidar_sdk` 削除の回避
- 完了: `platformio.ini` 変更の確認と反映
- 完了: 追加/移動されたリセット用ファイルの整合確認

## メモ
- マージではなく cherry-pick で取り込み済み。
- `platformio.ini` は `include_dir`/`data_dir`/`build_dir` を追加し、既存設定（Bluepad32差し替え・`lib_extra_dirs`）は維持。
