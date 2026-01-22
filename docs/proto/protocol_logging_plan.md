# プロトコルログ実装計画（ESP32 + RPi）

目的: 制御系を乱さずに UART プロトコルの送受信が正しいことを証明できるログを残す。  
対象: `docs/proto/overview.md` の UART プロトコル。

---

## 要件

- Serial2（制御UART）にテキストログを混ぜない。
- 低オーバーヘッド・ロス許容（制御優先）。
- RX/TX 成功と失敗理由（CRC/長さ/バージョン/未対応）を追える。
- RPi側で収集して後から解析できる。
- codec/フレーミング/異常系をテストで担保する。

---

## 設計概要

二段構えのログ:

1. **ローカル出力（ESP32 USB Serial / RPi stdout or file）**
   - 1秒毎の要約ログ（既定）
   - 必要時のみ詳細ログ（verbose）

2. **任意の統計フレーム（ESP32 -> RPi）**
   - デバッグ専用 `PROTO_STATS` を追加
   - 受信カウンタ・直近エラーを固定長で送る
   - 既存フレームと同様に安全に扱える

理由:
- ローカルログはUART2に影響しない。
- `PROTO_STATS` でESP32側の受信健全性をRPiで取得できる。

---

## ログ項目

詳細ログ（verbose時）:
- `RX_OK` type/seq/len/flags
- `RX_ERR` 原因（CRC/len/overflow/decode/ver）
- `TX_OK` type/seq/len/flags
- `TX_FAIL` type/seq
- `ACK` code/type_echo/seq_echo
- `STATUS` seq_applied/fault/age 等

要約ログ（1Hz既定）:
- 受信: `rx_ok`, `rx_crc`, `rx_len`, `rx_decode`, `rx_ver`, `rx_unsupported`
- 送信: `tx_ok`, `tx_fail`, `ack_ok`, `ack_err`, `status_rx`
- 直近: `last_rx_type/seq`, `last_tx_type/seq`, `last_err`

---

## ESP32 側 実装計画

1. `ProtoTrace` モジュール追加  
   - `firmware/src/comm/ProtoTrace.{h,cpp}`
   - API: `onRxFrame`, `onRxError`, `onTx`, `onAck`, `onStatus`, `tick()`
   - カウンタ/直近情報を保持

2. 既存パスへフック  
   - `PacketReader`: decode/CRC/len/overflowエラーを `onRxError`
   - `uart_rx_poll`: `kOk` で `onRxFrame`
   - `TxPort`: ACK/STATUS の送信成否を `onTx`

3. 出力  
   - USB `Serial` のみ使用（Serial2禁止）
   - 要約ログの間隔: `PROTO_TRACE_SUMMARY_MS`（既定 1000）
   - verbose は `PROTO_TRACE_VERBOSE` で切替

4. （必要なら）`PROTO_STATS` 実装  
   - `Protocol.h` に type追加
   - `ProtoStatsPayload` 定義
   - `TxPort::sendProtoStats` を要約間隔で送信

---

## RPi 側 実装計画

1. `ProtoTrace` モジュール追加  
   - `rpi/src/proto/Trace.{h,cpp}`
   - ESP32と同じカウンタ/要約

2. Sender/Receiver へフック  
   - `Sender::send` → `onTx`
   - `Sender::poll` → `onRxFrame` / `onRxError`
   - `ACK` / `STATUS` / `PROTO_STATS` を解析して詳細記録

3. 出力/保存  
   - CLIオプション想定: `--proto-log`, `--proto-log-file`
   - 既定は1Hz要約をstdout

---

## テスト計画（pytest方針）

### Unit（ホストC++）
- `rpi/test/proto_codec_test.cpp`
  - CRC16の既知ベクタ
  - COBS encode/decode ラウンドトリップ（0x00含む）
- `rpi/test/proto_reader_test.cpp`
  - 正常フレームのdecode
  - CRC不一致 / 長さ不一致 / バージョン不一致
- `rpi/test/proto_writer_test.cpp`
  - 既知のフレーム出力（ヘッダ+payloadのgolden）

### Integration（ホスト）
- `PacketWriter -> PacketReader` のループバック
- ランダム分割バイト列で再同期を検証

---

## 設定/フラグ

ESP32（`firmware/src/config/Config.h`）:
- `PROTO_TRACE_ENABLE`
- `PROTO_TRACE_VERBOSE`
- `PROTO_TRACE_SUMMARY_MS`

RPi:
- `PROTO_TRACE_VERBOSE=1`
- `PROTO_TRACE_LOGFILE=/path/to/log.txt`

---

## 実装順序

1. 両側に `ProtoTrace` とカウンタ追加（要約ログのみ）
2. 送受信フックを差し込み
3. verbose出力追加
4. 必要なら `PROTO_STATS` 追加
6. 実機UARTでログ取得・検証
