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
| flags  | 1    | bitfield（圧縮/ACK要求など） |
| level  | 1    | ログレベル（LOGのみ。CMD/STATEは0） |
| seq    | 2    | 送信側連番（ロス/順序判定） |
| t_us   | 4    | 送信側時刻（micros()低32bitでOK） |
| len    | 2    | payload長 |
| payload| N    | typeごとの構造体 |
| crc16  | 2    | CRC16-CCITT-FALSE |

---

## type設計（最低限）

### ESP32 -> RPi

- `0x10` LOG_EVENT: 構造化ログ
- `0x11` LOG_TEXT: テキストログ（デバッグ専用）
- `0x20` STATE_MIN: 最小状態（速度/ステア/fault/drop数）
- `0x21` STATE_DIAG: 診断（電流/温度/バッファ使用率など）

### RPi -> ESP32

- `0x01` CMD_DRIVE: 角度/速度/TTL/距離/モード
- `0x02` CMD_KILL: 非常停止（最優先）
- `0x03` CMD_LOGCFG: ログレベル/送信レート/有効type

---

## LOG_EVENT payload（構造化ログ）

最小構成の固定 + 可変に寄せる。

| Field     | Size | 説明 |
| --------- | ---: | ---- |
| event_id  | 2    | 例: 0x0001=CMD_RX, 0x0002=MOTOR_APPLY |
| component | 1    | 例: Drive=1, Engine=2, Steer=3, BT=4 |
| argc      | 1    | 引数個数 |
| args      | 4*argc | 各引数は int32（単位はevent定義側で固定） |

※ 拡張性重視なら TLV へ移行可能（未知tagはスキップ）。

---

## Logger設計（ESP32側）

### 役割分離（SOLID志向）

- LoggerCore: API、リングに push するだけ
- EventRegistry: type -> encoder 登録
- IEventEncoder: payload直列化
- UartLogTransport: LogTxTask側で送信

### 追記で増やすポイント

- `events/*.cpp` を追加して event_id を増やす
- 登録は `register_all_events(LoggerRegistry&)` で明示的に追加

---

## 収集すべき最低ログ

### 制御コマンド系

- CMD_RX / CMD_APPLY / FAILSAFE_TTL / KILL

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

## UART帯域の目安

- **921600 bps**
- 低重要度ログはサンプリング（STATEは20Hz、LINK_STATは1Hz）。

---

## 実装落とし込み（開始点）

### ESP32

- `firmware/src/comm/`
  - `FramingCobs.*`, `Crc16.*`, `Message.*`, `Dispatch.*`
- `firmware/src/log/`
  - `LoggerCore.*`, `UartLogTransport.*`, `events/*.cpp`

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
