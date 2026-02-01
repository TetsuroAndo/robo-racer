# RPi apps ログ出力整備 ステータス

## 目的
- rpi/apps の各プロセスでログファイルが自動生成される状態にする
- seriald と同等の FileSink 追加・ログパス整備を行う

## 進捗
- 準備中: なし
- 進行中: なし
- 再検討: なし
- 完了: metricsd / sim_esp32d / serialctl へ FileSink とログパス追加

## 差分メモ
- 既存: seriald は --log とデフォルトログパスあり
- 追加: 他 apps もデフォルトログパスと --log を導入
