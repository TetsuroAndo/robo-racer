# TSD20 firmware統合 実装ステータス

## 目的
- ESP32 firmwareにTSD20ドライバを統合し、I2C距離に基づく安全クランプを追加する。

## ステータス
- 進行中: 0
- 完了: 5
- 再検討: 0

## タスク一覧
- [x] 関連ドキュメント確認（firmware/proto/plan）
- [x] `hardware/Tsd20.{h,cpp}` 追加
- [x] `config/Config.h` にTSD20設定を追加
- [x] `main.cpp` に初期化と距離読取・クランプを追加
- [x] 動作ログの最小出力を追加
