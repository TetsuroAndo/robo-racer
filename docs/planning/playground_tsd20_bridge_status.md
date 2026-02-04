# playground/tsd20_bridge 実装ステータス

## 目的
- Arduino IDEで動いていたTSD20の自動I2C切替＋距離計測ブリッジを、
  本リポジトリの`playground/`でビルド可能にする。

## ステータス
- 進行中: 0
- 完了: 3
- 再検討: 1

## タスク一覧
- [x] `message.txt`内容の移植先構成を決定
- [x] `playground/tsd20_bridge`に実装を追加
- [x] `playground/tsd20_bridge/platformio.ini`を追加
- [ ] PlatformIOでビルド確認（`~/.platformio`の権限不足で失敗）

## メモ
- `platformio run -d playground/tsd20_bridge` 実行時、
  `/Users/atomboy/.platformio/`配下のlock作成に失敗してビルド不可。
