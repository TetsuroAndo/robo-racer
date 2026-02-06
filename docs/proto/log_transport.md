# UARTログ伝送アーキテクチャ（ESP32 -> RPi）

## 前提（2026-01-22 方針）

- ログはファイル保存し、RAMでもバッファリングする。
- ESP32は状態を極力持たず命令待機に寄せる。
- 非常停止（KILL/PANIC）を最優先する。

この文書は、現行の「Serial2 UART 制御／Bluepad32 手動入力」を保ちつつ、
**非ブロッキング志向・拡張可能なログ伝送**を統合する設計をまとめる。

---

## 目的

- 制御ループを絶対に止めない（ログは落ちても良い）。
- UARTストリーム上で堅牢に再同期できるフレーミングを採用する。
- type追加で拡張できる構造化ログを標準にする。
- RPi側で生ログを保存し、後から再解析可能にする。

---

## 現行実装との統合ポイント

- 制御は引き続き Serial2 UART を使用し、ログも同一UARTに混載する。
- Bluepad32 の手動入力は現行方針のまま（自律中は無視、KILLのみ有効）。
- KILL/PANIC は常に最優先で処理し、ログ送信よりも優先する。

---

## システム全体像

```
[ESP32]
  ControlTask (high)  ---->  log enqueue (drop OK)
  UartRxTask (high)   ---->  cmd store + log enqueue
  LogTxTask (low)     ---->  COBS + CRC -> Serial2

[UART: 921600 bps]

[RPi]
  seriald (sole owner)
    RX thread: decode + dispatch
    TX thread: cmd send
    log forward -> logd
  logd
    raw frame file + jsonl (optional)
```

---

## ESP32側: タスク分離（FreeRTOS）

### 1. ControlTask（最優先）

- UART受信済みコマンドを適用（Drive.setSpeed / setAngle）。
- TTL/kill/heartbeat でフェイルセーフ判定。
- 重要イベントは Logger に投げるだけ（I/O禁止）。

### 2. UartRxTask（高優先度）

- RPi -> ESP の受信フレームをデコード。
- CRC/len検証、typeディスパッチ。
- 受信イベント（CRC失敗など）をログ化。

### 3. LogTxTask（低優先度）

- ログ用リングバッファから取り出して UART 送信。
- `availableForWrite()` を見て小刻みに送る。
- 送れない場合はブロックせず drop（dropカウンタ加算）。

### 非ブロッキング原則

- `Logger::emit()` は O(1) で終了すること。
- ログI/Oは ControlTask から隔離すること。
- ログ送信の詰まりは **制御に影響させない**。

### 注意: 速度ランプのブロッキング

現状 `Engine::setSpeed()` は `delay()` を含むため制御ループを止める。
将来的には `SlewRateLimiter` による**非ブロッキング速度ランプ**へ移行すると、
ログ設計の価値が最大化する。

---

## RPi側: UART専用プロセス

### 推奨: seriald + logd

- **seriald**: UARTを唯一占有するプロセス。
  - RXスレッド: read -> COBS decode -> CRC verify -> type dispatch
  - TXスレッド: cmd queue -> write
  - ログは logd へ転送（UNIXドメインソケット等）
- **logd**: ファイル保存、JSONL生成、リアルタイム配信。

### 最小構成（今すぐ動かす）

- 同一プロセス内に2スレッド（RX/TX）を置く。
- 将来の保守性・リアルタイム性は seriald 分離が有利。

---

## フレーミングとパケット構造

### フレーミング: COBS + 0x00 終端

- UARTはバイトストリームなので境界が必要。
- COBSで `0x00` を排除し、終端 `0x00` でフレーム確定。
- 破損しても次の `0x00` から復帰できる。

### CRC

- CRC-16/CCITT-FALSE 推奨。
- CRCは末尾に置く（len分読んで一括検証）。

### ベースフレーム（COBS前の生データ）

| Field  | Size | 説明 |
| ------ | ---: | ---- |
| magic  | 2    | 固定 `0x4D 0x43` ("MC") |
| ver    | 1    | プロトコル版（例: 1） |
| type   | 1    | メッセージ種別 |
| flags  | 1    | bit0: ACK_REQ（要求側がACKを要求） |
| seq    | 2    | 送信側連番（ロス/順序判定） |
| len    | 2    | payload長 |
| payload| N    | typeごとの構造体 |
| crc16  | 2    | CRC16-CCITT-FALSE |

※ v1では `flags` の bit0（ACK_REQ）のみ使用。その他は予約（0固定）。

---

## type設計（最低限）

### ESP32 -> RPi

- `0x10` LOG: テキストログ（level + text）
- `0x11` STATUS: 状態（速度/ステア/fault/age）
- `0x80` ACK: ACK応答（payloadなし）

### RPi -> ESP32

- `0x01` DRIVE: 角度/速度/TTL/距離
- `0x02` KILL: 非常停止（最優先）
- `0x03` MODE_SET: MANUAL/AUTO 切替
- `0x04` PING: 生存確認（ACK応答）

---

## LOG payload（テキストログ）

| Field | Size | 説明 |
| ----- | ---: | ---- |
| level | 1    | ログレベル（0=TRACE..5=FATAL） |
| text  | N    | UTF-8 テキスト（可変長） |

※ 将来的に構造化ログへ拡張する場合は別typeで追加する。

## STATUS payload（状態通知）

| Field | Size | 説明 |
| ----- | ---: | ---- |
| seq_applied | 1 | 最後に適用した `DRIVE.seq`（下位8bit） |
| auto_active | 1 | 1=自律有効 |
| faults | 2 | bitfield（KILL/HB_TIMEOUT/TTL_EXPIRED/AUTO_INACTIVE） |
| speed_mm_s | 2 | 現在速度（i16 LE） |
| steer_cdeg | 2 | 現在ステア（i16 LE） |
| age_ms | 2 | 最終コマンドからの経過（u16 LE, 0xFFFFで飽和） |

## ACK（応答）

- 受信側は `flags & ACK_REQ` のとき **ACKを返す**。
- `ACK` は `type=0x80`、`payload_len=0`、`seq` は要求側の `seq` をそのまま返す。
- **PINGは常にACK**（`ACK_REQ` に依存しない）。ただし len!=0 は不正として ACK しない。

---

## Logger設計（ESP32側）

### 役割分離（現行）

- AsyncLogger: level + text をフレーム化して enqueue
- UartTx: LogTxTask側で送信

### 追記で増やすポイント

- 構造化ログを別typeで追加し、専用のencoderを用意

---

## 収集すべき最低ログ

### 制御コマンド系

- DRIVE_RX / DRIVE_APPLY / FAILSAFE_TTL / KILL

### モータ/ステア出力

- MOTOR_APPLY / STEER_APPLY

### 通信健全性

- UART_RX_ERR / LOG_DROP / LINK_STAT

### Bluetooth

- BT_CONN / MANUAL_IGNORED

---

## RPi側ログ保存フォーマット

1. **生フレーム保存**
   - COBS復号後のフレーム列をそのまま保存
   - 将来デコーダ差し替えが可能
2. **JSON Lines（拡張）**
   - 重要イベントのみJSON化して保存

`spdlog::async_logger` + ローテーションを推奨。

---

## UART帯域の目安 v1

- **UART 921600 bps / 8N1**
- 制御・状態系の代表的な送信レート（`docs/proto/overview.md` に準拠）:
  - `DRIVE`: 100Hz（10ms）
  - `STATUS`: 100Hz（10ms）
  - `IMU_STATUS`: 100Hz（10ms）
  - `TSD20_STATUS`: 100Hz（10ms）
  - `PING`: 10〜50Hz
- 低重要度ログ（`LOG` type 等）はサンプリング（間引き）し、帯域の大半を
  **DRIVE/STATUS/IMU/TSD20** に確保する。

---

## 実装落とし込み（開始点）

### ESP32

- `firmware/src/comm/`
  - `mc_proto.*`, `UartTx.*`
- `firmware/src/log/`
  - `AsyncLogger.*`

### RPi

- `rpi/seriald/`（推奨）
- `rpi/logd/`

---

## 固定パラメータ

- `DEFAULT_ESP_BAUD = 921600`
- UARTは RPi 側で seriald が単独所有

---

## まとめ

- **制御最優先**、ログは drop して良い。
- **COBS + CRC + type-dispatch** が拡張の核。
- ESP32はタスク分離で非ブロッキング化。
- RPiは seriald/logd 分離で I/O を隔離。
