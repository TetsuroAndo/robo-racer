# プロトコル テスト計画（ESP32 + RPi）

## 1. 対象範囲
現行の mc_proto 実装に対して以下を確認する。
- フレーミング（COBS + CRC16）
- ヘッダ検証
- typeごとのpayload仕様
- ESP32側ハンドラの挙動
- RPi側 Sender/seriald の挙動
- UART実機でのE2E

## 2. 現行プロトコルのパターン

### 2.1 フレーム形式
- 生フレーム: `Header + payload + crc16` を COBS エンコードし、末尾に `0x00` 区切り
- Header（9 bytes）
  - `magic[2] = 'M','C'`
  - `ver = 1`
  - `type = u8`
  - `flags = u8`（`ACK_REQ=0x01` のみ、他はv1未使用）
  - `seq = u16 LE`
  - `len = u16 LE`
- CRC16-CCITT（poly 0x1021, init 0xFFFF）を `Header + payload` に対して計算

### 2.2 type 定義（現行）
- RPi -> ESP32
  - `0x01 DRIVE`（len=8）
  - `0x02 KILL`（len無視）
  - `0x03 MODE_SET`（len>=1許容）
  - `0x04 PING`（len無視）
- ESP32 -> RPi
  - `0x10 LOG`（len>=1）
  - `0x11 STATUS`（len=10）
  - `0x80 ACK`（len=0、flags=0、seqをエコー）

### 2.3 payload 定義
- DRIVE（8 bytes）
  - `steer_cdeg: i16 LE`
  - `speed_mm_s: i16 LE`
  - `ttl_ms: u16 LE`
  - `distance_mm: u16 LE`
- MODE_SET（>=1 byte）
  - `mode: u8`（0=MANUAL, 1=AUTO）
  - `reason: u8`（任意、無視される）
- PING
  - payloadは無視（現在の実装）
- KILL
  - payloadは無視（現在の実装）
- STATUS（10 bytes）
  - `seq_applied: u8`（last_seq下位8bit）
  - `auto_active: u8`
  - `faults: u16 LE`（bit0 KILL, bit1 HB_TIMEOUT, bit2 TTL_EXPIRED, bit3 AUTO_INACTIVE）
  - `speed_mm_s: i16 LE`
  - `steer_cdeg: i16 LE`
  - `age_ms: u16 LE`
- LOG（>=1 byte）
  - `level: u8 (0..5)`
  - `text: UTF-8文字列`（可変長、空でも可）

## 3. テスト戦略概要
- ユニット: codec / PacketReader / PacketWriter
- コンポーネント: ESP32ハンドラ、RPi Sender/seriald の挙動
- 統合: RPi内ループバック（seriald + Sender）
- HIL: 実UARTでESP32 <-> RPi

## 4. テストマトリクス（パターン一覧）

### 4.1 フレーミング / Codec（共通）
**正常系**
- COBSの往復（0x00含む入力）
- len=0 の最小フレーム
- len=MAX_PAYLOAD の最大フレーム
- 連続フレーム（バックトゥバック）

**異常系**
- CRC不一致
- COBSデコード失敗（code=0、切れた入力）
- magic不一致
- ver不一致
- len>MAX_PAYLOAD
- lenとpayload長の不一致
- encoded長オーバー

### 4.2 ヘッダ/flags
- FLAG_ACK_REQ 有効時に ACK を返すこと（PINGは常にACK）
- ACKフレームは `payload_len=0` かつ `flags=0` であること
- 未対応type受信時にクラッシュしないこと

### 4.3 RPi -> ESP32 type
**DRIVE**
- len=8 正常
- len!=8 エラー
- steer/speedのクランプ
- ttlのクランプ（最小/最大）
- distance保持
- last_seq/last_cmd_ms/expireの更新

**MODE_SET**
- len=1 mode=0 -> MANUAL
- len=1 mode=1 -> AUTO
- len=2 reason付き -> mode反映
- mode=2等の不正値 -> エラー

**PING**
- len=0 で last_hb_ms 更新 + ACK送信
- len=4 でも同様に動作

**KILL**
- len=0 で killed=true, cmd_expire=0
- len=2 でも同様に動作

### 4.4 ESP32 -> RPi type
**STATUS**
- len=10 検証
- seq下位8bit反映
- faultsビットの反映
- age_msの0xFFFFサチュレート

**LOG**
- len=1（textなし）で空ログ
- len>1でtext出力
- level範囲のクランプ（0..5）

**ACK**
- payload_len=0
- flags=0
- seqが要求側と一致

## 5. ESP32側テスト計画

### 5.1 ハンドラ単体（ホストビルド）
- Context/ControlState/Logger/UartTx をモック化
- DriveHandler: 各state更新とクランプ確認
- ModeHandler: mode切替・不正値エラー
- KillHandler: killラッチ/expire
- PingHandler: last_hb_ms更新、ACK enqueue

### 5.2 HIL統合
- UARTでフレーム送信
- STATUSを見て以下を確認
  - AUTOでDRIVEが反映される
  - TTL切れでTTL_EXPIRED
  - PING停止でHB_TIMEOUT
  - KILLでKILL_LATCHED

## 6. RPi側テスト計画

### 6.1 Codec拡張
- `test/rpi/proto/cpp/proto_tests.cpp` を拡張
  - max payload
  - multi-frame
  - resync
  - len不一致

### 6.2 Senderテスト
- auto_disabled時にDRIVE送信しない
- PING送信間隔
- MODE_SET送信

### 6.3 serialdテスト
- LOG payload（level+text）の表示
- STATUS表示
- IPC->UARTの透過転送

## 7. E2E（RPi <-> ESP32）

### 7.1 通常制御
1) MODE_SET(AUTO)
2) DRIVE 50-100Hz
3) STATUSでspeed/steer/age確認

### 7.2 フェイルセーフ
- DRIVE停止 -> TTL_EXPIRED
- PING停止 -> HB_TIMEOUT
- KILL送信 -> KILL_LATCHED

### 7.3 ログ
- ESP32ログ発火
- serialdでテキスト出力
- 制御ループが阻害されないこと

## 8. テスト配置案
- 既存: `test/rpi/proto/cpp/proto_tests.cpp`, `test/firmware/proto/cpp/proto_tests.cpp`
- 新規案: `test/firmware/comm/test_handlers.cpp`
- 新規案: `test/rpi/comm/test_sender.py`

## 9. 受け入れ条件
- ホスト側テストが全て通る
- HILでSTATUS/LOGの期待出力が得られる
- seriald転送に回帰がない
