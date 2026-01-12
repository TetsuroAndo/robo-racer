# LOG/HILS 運用ガイド（Guide）

このドキュメントは、LOGの使い方、設定方法、運用パターンを
まとめた実務向けガイドです。

## 設定ファイル

設定は `firmware/src/config/Config.h` にあります。

### ログ設定

- `LOG_LEVEL_DEFAULT`
- `LOG_LEVEL_COMM`, `LOG_LEVEL_CONTROL`, `LOG_LEVEL_HARDWARE`,
  `LOG_LEVEL_HILS`
- `LOG_TO_SERIAL`（テキストログをUARTへ）
- `LOG_TO_BLE`（テキストログをBLEへ）
- `LOG_TO_UART_PROTOCOL`（COBS+CRCログをUARTへ）
- `LOG_TO_UDP`（UDPログを有効化）

HILS設定は `docs/firmware/hils_guide.md` を参照してください。

### UDP設定

- `WIFI_SSID`, `WIFI_PASS`
- `LOG_UDP_HOST`, `LOG_UDP_PORT`
- `LOG_UDP_CONNECT_TIMEOUT_MS`

## 推奨運用パターン

### UARTにCOBSログ、制御はUARTのまま

```
LOG_TO_SERIAL = false
LOG_TO_UART_PROTOCOL = true
LOG_TO_UDP = false
```

RPi側でCOBS+CRCをデコードし、`MsgType::Log` を処理します。

### 高頻度ログはUDP、制御はUART固定

```
LOG_TO_SERIAL = false
LOG_TO_UART_PROTOCOL = false
LOG_TO_UDP = true
WIFI_SSID / WIFI_PASS / LOG_UDP_HOST / LOG_UDP_PORT を設定
```

UARTは制御専用、ログはUDPに分離されます。

### デバッグ時の軽量テキストログ

```
LOG_TO_SERIAL = true
LOG_TO_UART_PROTOCOL = false
LOG_TO_UDP = false
```

## ログAPIの使い方

### レベルとトピック

レベル:

- `Trace`, `Debug`, `Info`, `Warn`, `Error`, `Off`

トピック:

- `App`, `Comm`, `Control`, `Hardware`, `Hils`

### マクロ

```
MC_LOGT(topic, "fmt", ...)
MC_LOGD(topic, "fmt", ...)
MC_LOGI(topic, "fmt", ...)
MC_LOGW(topic, "fmt", ...)
MC_LOGE(topic, "fmt", ...)
```

例:

```
MC_LOGI(mc::log::Topic::Comm, "BLE connected");
```

### Logger API（概要）

- `Logger::attachSink(Sink*)`
- `Logger::setLevelAll(Level)`
- `Logger::setLevel(Topic, Level)`
- `Logger::enabled(Topic, Level)`
- `Logger::logf(Topic, Level, const char *fmt, ...)`

## 運用上の注意点

- UARTのCOBSログとテキストログは混在させない
- UDPログはWi-Fi接続失敗時に無効化される
- 文字列ログは最大96文字に切り詰められる
- Sinkは最大3つまで

## 受信側（RPi）実装のヒント

- UART/UDPともにCOBS+CRCで同じデコーダを使える
- `MsgType::Log` は `LogPayloadHeader` + メッセージバイト列
- `msg_len` を信頼し、NUL終端はない前提で扱う

HILSの運用は `docs/firmware/hils_guide.md` を参照してください。
