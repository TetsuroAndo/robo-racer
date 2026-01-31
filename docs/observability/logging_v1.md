# ロギング v1（Interface Freeze）

## 目的
- 制御ループを止めずにログを残す
- コマンド/状態遷移を必ず記録し、後から再現可能にする

## 正本
- ESP32側ログ転送: `docs/proto/log_transport.md`
- RPi側ロガー: `rpi/lib/mc_core/include/mc/core/Log.hpp`

## RPi側 Logger（v1固定）
- API: `mc::core::Logger` を全プロセスで共通利用
- レコード: `LogRecord { ts_us, level, tag, msg }`
- 非同期: 内部キューで非同期書き込み（dropは古いものから）
- 既定の出力: ConsoleSink（stderr）
- 追加出力: FileSink（追記）

### 文字列フォーマット（FileSink/ConsoleSink）
```
[<ts_us>] <LEVEL> [<tag>] <msg>
```

## ESP32↔RPi ログ転送（UART）
- type: `LOG (0x10)`
- payload: `level + text (UTF-8)`
- 非同期送信、送信詰まり時はdrop可

## 記録対象（最低限）
- MODE_SET/DRIVE/KILL などの**命令内容**
- 状態遷移（AUTO/MANUAL、KILLラッチ、TTL失効）
- 重要イベント（CRC不正、HBタイムアウト、UART切断）

## ローテーション/容量（v1方針）
- v1では**ローテーションは未実装**
- 運用では `logrotate` 等の外部管理で回す

## Dropポリシー（v1固定）
- RPi: `Logger` 内部キューは上限を超えたら **古いレコードを破棄**
- ESP32: LogTxTask 送信詰まり時は **送信を諦めてdrop**

