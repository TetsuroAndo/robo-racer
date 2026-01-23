方針
* **Bluetooth（Bluepad32）は ESP32 に直結**
* **自律中は手動入力を受けない（介入は KILL のみ）**
* **ESP32 は “プロトコル中心”に拡張可能（type 追加で機能追加）**
* **低遅延かつ安定（フレーム化＋CRC、誤同期復帰が強い）**
**詳細なプロトコル仕様書（実装可能な粒度）**と、**ESP32 側の実装設計（ポリモーフィズム＝登録型ディスパッチ＋入力ソース差し替え可能）**

---

# 1. 用語・前提

* **RPi**：上位（自律・SLAM・経路・安全監視など）。ESP32へ制御命令送信。
* **ESP32**：下位（受信した命令を適用、フェイルセーフ、KILL）。
* **MANUAL**：ESP32のBluetoothゲームパッド入力（Bluepad32）。
* **AUTO**：RPiからUARTで送られる自律制御コマンド。
* **KILL**：緊急停止。**どのモードでも最優先**。解除は現状プロトコルでは行わない（再起動/ローカル操作）。

---

# 2. システム方針（最重要）

## 2.1 モードポリシー（確定）

* **AUTOモード（mode=AUTO）**：

  * Bluetooth 手動入力は **無視**
  * 介入できるのは **KILL のみ**（BT でも UART でも可）
* **MANUALモード（mode=MANUAL）**：

  * Bluetooth 手動入力を Drive に反映して良い
  * UART の DRIVE コマンドを受けても適用しない（受理のみ）

※この「自律中は手動無視」を成立させるため、**MODE_SET でモードを明示切り替え**します（後述）。

---

# 3. 通信プロトコル（UART）仕様

## 3.1 要求

* 低遅延
* 誤同期に強い（すぐ復帰）
* 拡張容易（type追加＝ハンドラ追加）
* CRCで破損検知
* 最新のみ採用（seq）
* 期限切れは停止（TTL）

## 3.2 フレーム方式：COBS + delimiter + CRC16

### 3.2.1 物理レイヤ

* UART（921600 bps）
* 8N1

### 3.2.2 フレーム境界

* **フレーム終端（delimiter）**：`0x00`
* 受信は `0x00` を見つけたら「1フレーム確定」とする

### 3.2.3 COBS

* `header + payload + crc16` を COBS エンコードし、最後に `0x00` を付与
* 受信側は `0x00` までを COBS デコードする

### 3.2.4 CRC

* CRC16-CCITT（多項式 0x1021, 初期値 0xFFFF 推奨）
* 対象：**header(MAGIC〜LEN) + payload**
* CRCは little-endian 2byte を末尾に付与

---

## 3.3 パケット構造

### 3.3.1 Header v1（固定 9 bytes）

| フィールド |      型 | 説明                       |
| ------- | -----: | --------------------------- |
| `magic` |   u8×2 | 固定 `0x4D 0x43` ("MC")     |
| `ver`   |     u8 | 現行は 1                     |
| `type`  |     u8 | メッセージ種別                |
| `flags` |     u8 | フラグ（共通）                |
| `seq`   | u16 LE | シーケンス（0-65535循環）      |
| `len`   | u16 LE | payload長（0..MAX_PAYLOAD）  |

**LE変換ルール（重要）**

* `seq` / `len` は **wire上は little-endian**。受信側は必ず変換して扱う。
* Pi/ESP32がともにlittle-endianでも、**仕様として rd16_le()/wr16_le() を通す**こと。
* “たまたま動く”実装を防ぐため、サンプルコードも変換込みで示す。

**flags（共通）定義**

* bit0: `ACK_REQ`（1=ACKが欲しい）
* bit1: `RESERVED`
* bit2: `RESERVED`
* bit3..7: 予約

> MANUAL/AUTO は flags では表しません。理由：あなたの確定方針が「mode=AUTO時は手動無視」なので、入力の“出所”は ESP32 内部のソース（BT）と UART（RPi）で明確に分離でき、flags に混ぜると事故るためです。

---

## 3.4 type 定義（v1）

| type | 名前              | 方向      | payload  | 目的                             |
| ---: | ---------------- | --------- | -------- | -----------------------         |
| 0x01 | `DRIVE`          | RPi→ESP   | 固定      | 操舵/速度/TTL                     |
| 0x02 | `KILL`           | RPi→ESP   | なし      | 即時停止（ラッチ）                |
| 0x03 | `MODE_SET`       | RPi→ESP   | 固定      | MANUAL/AUTO 切替                  |
| 0x04 | `PING`           | RPi→ESP   | なし      | 生存確認（ACK応答）                |
| 0x10 | `LOG`            | ESP→RPi   | 可変      | ログ（level + text）               |
| 0x11 | `STATUS`         | ESP→RPi   | 固定      | 状態通知（任意）                  |
| 0x80 | `ACK`            | ESP→RPi   | なし      | PING応答                          |

---

## 3.5 payload 定義

### 3.5.1 `MODE_SET` (type=0x03) payload（1 bytes）

| フィールド    |  型 | 説明                          |
| -------- | -: | --------------------------- |
| `mode`   | u8 | 0=MANUAL, 1=AUTO             |

**挙動**

* mode=1（AUTO）：以後、UARTの `DRIVE` を適用対象とする。BT入力は無視。
* mode=0（MANUAL）：以後、UARTの `DRIVE` は受理のみ。BT入力が適用対象。

> 自律中に手動を受けないための「唯一の切替」です。RPiが制御権を握っていることを明示します。

**v1 固定仕様**

* `len == 1` のみ許可。`len != 1` は無視（or NACK）。
* `reason` は v2 で追加する（例：`MODE_SET_EX` など）。

---

### 3.5.2 `DRIVE` (type=0x01) payload（8 bytes）

物理量寄りで固定し、後からPIDや車両スケールが変わっても仕様が安定するようにします。

| フィールド      |  型 | 単位    | 説明                                                |
| ------------- | --: | ------ | --------------------------------------------------- |
| `steer_cdeg`  | i16 | 0.01°  | 目標舵角（0中心、左+右- などは実装で統一）                |
| `speed_mm_s`  | i16 | mm/s   | 目標速度（符号付き。車体/モータ都合で上限は実装側でクランプ） |
| `ttl_ms`      | u16 | ms     | このコマンドの有効期限                                  |
| `distance_mm` | u16 | mm     | 将来拡張用（0なら無視）                                 |

**TTLの評価基準**

* ESP32 がこのフレームを **受理（CRC OK）した時刻 `t_rx_ms`** を起点に、

  * `now_ms - t_rx_ms > ttl_ms` なら失効 → STOP
* RPi時刻 `t_ms` は使わない（あなたの要望）

**distance_mm の扱い**

* v1では 0 以外でも “参照値”として保持して良い
* エンコーダ導入後に「距離到達で停止」へ拡張可能

---

### 3.5.3 `PING` (type=0x04) payload（0 bytes）

* len=0 固定（len!=0 は不正として無視）
* 受理したら `last_hb_ms=now` を更新（payloadは参照しない）
* 受理時に `ACK` を返す（payloadなし、headerのseqで応答）
* **PINGは常にACK**（`FLAG_ACK_REQ` に依存しない、ただし len!=0 は ACK しない）

運用メモ：
* RPi は **modeに関わらず** PING を送る（リンク監視目的）

**v2 で nonce を入れるなら** `PING_EX` を追加する。

---

### 3.5.4 `KILL` payload なし

* KILL受理：`kill_latched=true` にする（必ずラッチ）
* 解除は現状プロトコルでは行わない（安全のためローカル操作/再起動）

---

### 3.5.5 `ACK` (type=0x80) payload なし

* **ACK要求**：request側が `FLAG_ACK_REQ` を立てる
* **ACK応答**：`type=ACK`、`flags=0`、payloadなし
* headerの `seq` は受信フレームの `seq` と一致させる
* payloadは将来拡張用（現状は空）

---

### 3.5.6 `STATUS` (type=0x11) payload（10 bytes）

| フィールド         |  型 | 単位       | 説明                          |
| ---------------- | --: | -------- | ---------------------------------- |
| `seq_applied`    |  u8 | -        | 最後に適用した `DRIVE.seq`（下位8bit） |
| `auto_active`    |  u8 | -        | 1/0                                |
| `fault`          | u16 | bitfield | 故障/停止要因                        |
| `speed_now_mm_s` | i16 | mm/s     | 現在速度（暫定は指令値でも良い）       |
| `steer_now_cdeg` | i16 | 0.01°    | 現在舵角（暫定は指令値でも良い）        |
| `age_ms`         | u16 | ms       | 最終DRIVE受理からの経過       |

※ `seq_applied` は `seq` の下位8bitを載せる運用でも可。

**fault bitfield（v1案）**

* bit0: KILL_LATCHED
* bit1: HB_TIMEOUT
* bit2: TTL_EXPIRED
* bit3: AUTO_INACTIVE（MODE=MANUALなのにDRIVEが来ている等、運用監視）
* bit4..15: 予約

---

### 3.5.7 `LOG` (type=0x10) payload（可変長）

| フィールド |  型 | 説明 |
| ------ | -: | ---- |
| `level` | u8 | ログレベル（0=TRACE..5=FATAL） |
| `text`  | u8[N] | UTF-8テキスト |

* payloadの先頭1 byteが `level`、残りがテキスト
* テキストは **NUL終端しない**（`len - 1` バイトが本文）
* テキストは空でも可
* 構造化ログは別type（例: `LOG_EVENT`）で拡張する

---

## 3.6 タイムアウト・周波数（推奨値）

* RPi→ESP32 `DRIVE`：50〜200Hz
* `ttl_ms`：制御周期の2〜3倍（例：送信100Hzなら ttl=30〜50ms）
* `PING`：10〜50Hz
* `hb_timeout_ms`（ESP32内部）：200ms（暫定）
* `STATUS`：20Hz（必要なときだけ）

---

# 4. ESP32 制御ロジック仕様（安全優先順位）

制御ループ（例：200Hz = 5ms周期）で必ずこの順で評価します。

1. **KILL**：`kill_latched==true` → `Engine.stop()` + `Steer.center()`（または安全角）
2. **mode==AUTO のとき**：

   * `hb_timeout`（HB使うなら）→ STOP
   * `ttl_expired` → STOP
   * 有効なら `DRIVE` を適用
3. **mode==MANUAL のとき**：

   * Bluetooth入力を適用（ただし KILL 優先）

> これにより「自律中は手動無視（介入はKILLのみ）」がプロトコルレベルで崩れません。

---

# 5. ESP32 実装設計（拡張性＝type追加だけで機能追加）

あなたの現行ツリーを尊重し、最小の侵襲で移行できる形にします。

## 5.1 追加するコンポーネント（提案）

* `comm/mc_proto.*`：ヘッダ構造体、type enum、COBS/CRC/reader/writer
* `comm/registry.*`：type→handler 登録（登録型ディスパッチ）
* `comm/handlers/*.cpp`：DRIVE/MODE_SET/PING/KILL
* `comm/registry.h`：ControlState（mode/killed/targets/timers）
* `control/ControllerInput.*`：BT入力
* `hardware/Drive.*`：実際の駆動適用

## 5.2 “ポリモーフィズム”

組み込みで最も堅牢かつ軽いのは **登録型ディスパッチ（関数テーブル）**です。

* `HandlerFn handlers[256]`
* `register_handler(type, fn)`
* 追加コマンドは `handlers/new_type.cpp` を増やして登録するだけ

仮想関数でもよいですが、今回の要件（低遅延・コンパクト）なら関数テーブルが適します。

---

# 6. 参考実装（ESP32側の骨格コード）

以下は “仕様書を実装に落とすとこうなる” という最小骨格です。
（あなたのリポジトリ構造に合わせたパスは例です。実際に配置する際は `src/comm/` などに入れてください。）

## 6.1 Protocol 定義（例：`src/comm/mc_proto.h`）

```cpp
#pragma once
#include <stdint.h>

namespace proto {

static constexpr uint8_t VER = 1;
static constexpr size_t MAX_PAYLOAD = 64; // 必要に応じて

enum class Type : uint8_t {
  DRIVE    = 0x01,
  KILL     = 0x02,
  MODE_SET = 0x03,
  PING     = 0x04,
  LOG      = 0x10,
  STATUS   = 0x11,
  ACK      = 0x80,
};

enum : uint8_t {
  FLAG_ACK_REQ = 1u << 0,
};

#pragma pack(push, 1)
struct Header {
  uint8_t  magic[2];
  uint8_t  ver;
  uint8_t  type;
  uint8_t  flags;
  uint16_t seq_le;
  uint16_t len_le;  // little-endian on wire
};

struct ModeSetPayload {
  uint8_t mode;   // 0=MANUAL, 1=AUTO
};

struct DrivePayload {
  int16_t  steer_cdeg;   // 0.01 deg
  int16_t  speed_mm_s;   // mm/s
  uint16_t ttl_ms;
  uint16_t distance_mm;  // 0なら無視
};

struct LogPayload {
  uint8_t level;
  // uint8_t text[]; // UTF-8 text follows (variable length)
};

struct StatusPayload {
  uint8_t  seq_applied;
  uint8_t  auto_active;
  uint16_t fault;
  int16_t  speed_now_mm_s;
  int16_t  steer_now_cdeg;
  uint16_t age_ms;
};
#pragma pack(pop)

} // namespace proto
```

## 6.2 SafetyState（例：`src/control/SafetyState.h`）

```cpp
#pragma once
#include <stdint.h>

struct SafetyState {
  bool kill_latched = false;
  bool auto_active  = false;

  uint32_t last_hb_ms = 0;
  uint32_t hb_timeout_ms = 200;

  uint16_t fault = 0;

  enum FaultBits : uint16_t {
    KILL_LATCHED = 1u << 0,
    HB_TIMEOUT   = 1u << 1,
    TTL_EXPIRED  = 1u << 2,
    AUTO_INACTIVE= 1u << 3,
  };

  void updateFaults() {
    fault = 0;
    if (kill_latched) fault |= KILL_LATCHED;
    // HB_TIMEOUT, TTL_EXPIRED は判定側で立てる（瞬間値）
  }
};
```

## 6.3 DriveCommandStore（例：`src/control/DriveCommandStore.h`）

```cpp
#pragma once
#include <stdint.h>
#include "../comm/mc_proto.h"

struct DriveCommandStore {
  bool     has = false;
  uint16_t seq = 0;
  proto::DrivePayload sp{};
  uint32_t rx_ms = 0;

  void set(uint16_t s, const proto::DrivePayload& p, uint32_t now_ms) {
    has = true;
    seq = s;
    sp = p;
    rx_ms = now_ms;
  }

  bool ttlExpired(uint32_t now_ms) const {
    if (!has) return true;
    return (uint32_t)(now_ms - rx_ms) > (uint32_t)sp.ttl_ms;
  }

  uint16_t ageMs(uint32_t now_ms) const {
    if (!has) return 0xFFFF;
    return (uint16_t)(now_ms - rx_ms);
  }
};
```

## 6.4 Dispatcher（例：`src/comm/registry.h`）

```cpp
#pragma once
#include <stdint.h>
#include <stddef.h>

struct FrameView {
  const uint8_t* data; // points to Header
  size_t len;          // header+payload bytes (no crc)
};

struct Context; // forward

using HandlerFn = bool(*)(const FrameView&, Context&);

struct Dispatcher {
  HandlerFn table[256]{};

  void reg(uint8_t type, HandlerFn fn) { table[type] = fn; }

  bool dispatch(uint8_t type, const FrameView& f, Context& ctx) {
    auto fn = table[type];
    if (!fn) return false;
    return fn(f, ctx);
  }
};
```

## 6.5 Context（例：`src/comm/registry.h`）

```cpp
#pragma once
#include "../hardware/Drive.h"
#include "../control/SafetyState.h"
#include "../control/DriveCommandStore.h"

struct TxPort {
  // ACK/STATUS を返したくなった時に実装
  void sendAck(uint16_t seq);
};

struct Context {
  Drive& drive;
  SafetyState& safety;
  DriveCommandStore& autoCmd;
  TxPort& tx;
};
```

## 6.6 ハンドラ例：MODE_SET（例：`src/comm/handlers/ModeSetHandler.cpp`）

```cpp
#include "../mc_proto.h"
#include "../Dispatcher.h"
#include "../Context.h"
#include <string.h>

static inline const proto::Header* hdr(const FrameView& f) {
  return reinterpret_cast<const proto::Header*>(f.data);
}
static inline uint16_t rd16_le(uint16_t v_le) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return v_le;
#else
  return (uint16_t)((v_le >> 8) | (v_le << 8));
#endif
}

bool handleModeSet(const FrameView& f, Context& ctx) {
  const auto* h = hdr(f);
  const uint16_t len = rd16_le(h->len_le);
  if (len < 1) {
    return true;
  }
  proto::ModeSetPayload p{};
  const size_t copy_len = (len >= sizeof(p) ? sizeof(p) : 1);
  memcpy(&p, f.data + sizeof(proto::Header), copy_len);

  ctx.safety.auto_active = (p.mode != 0);
  return true;
}
```

## 6.7 ハンドラ例：DRIVE（例：`src/comm/handlers/DriveHandler.cpp`）

```cpp
#include "../mc_proto.h"
#include "../Dispatcher.h"
#include "../Context.h"
#include "../../lib/common/Time.h"
#include <string.h>

static inline const proto::Header* hdr(const FrameView& f) {
  return reinterpret_cast<const proto::Header*>(f.data);
}
static inline uint16_t rd16_le(uint16_t v_le) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return v_le;
#else
  return (uint16_t)((v_le >> 8) | (v_le << 8));
#endif
}

bool handleDrive(const FrameView& f, Context& ctx) {
  const auto* h = hdr(f);
  const uint16_t len = rd16_le(h->len_le);
  if (len != sizeof(proto::DrivePayload)) {
    return true;
  }

  proto::DrivePayload p{};
  memcpy(&p, f.data + sizeof(proto::Header), sizeof(p));

  const uint32_t now = mc::Time::ms();
  ctx.autoCmd.set(rd16_le(h->seq_le), p, now);
  return true;
}
```

## 6.8 ハンドラ例：KILL（例：`src/comm/handlers/KillHandler.cpp`）

```cpp
#include "../mc_proto.h"
#include "../Dispatcher.h"
#include "../Context.h"

static inline const proto::Header* hdr(const FrameView& f) {
  return reinterpret_cast<const proto::Header*>(f.data);
}
static inline uint16_t rd16_le(uint16_t v_le) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return v_le;
#else
  return (uint16_t)((v_le >> 8) | (v_le << 8));
#endif
}

bool handleKill(const FrameView& f, Context& ctx) {
  const auto* h = hdr(f);
  const uint16_t len = rd16_le(h->len_le);
  if (len != 0) {
    return true;
  }
  ctx.safety.kill_latched = true;
  return true;
}
```

## 6.9 main の制御ループ（現行 CSV 受信を置換する指針）

あなたの `main.cpp` は、次の3ブロックに整理します：

1. `uart_rx_poll()`：Serial2 から受信し、フレーム抽出→ディスパッチ
2. `bt_poll()`：Bluepad32 で pad 更新（ただし mode=AUTO なら適用しない。KILLボタンだけは許可）
3. `apply_control()`：安全優先順位で Drive に反映→ `drive.control()`

（COBSフレーマやCRC実装は長くなるのでここでは割愛しましたが、仕様は上記の通りです。）

---

# 7. Bluetooth（手動）入力の扱い仕様

## 7.1 自律中は手動無視（ただしKILLは許可）

* `mode==AUTO`：

  * `pad.update()` はしてもよい（接続維持・状態確認）
  * 走行コマンド（steer/speed）は **適用しない**
  * ただし **特定ボタン（例：HOME/START長押し）でKILLを発火**するのは許可（“介入はKILLのみ”の要件に一致）

## 7.2 自律でないときは手動操作OK

* `mode==MANUAL`：

  * padから `Drive.setSpeed()`/`Drive.setAngle()` を更新して良い
  * `Drive` の timeout は手動でも更新される（安全停止）

---

# 8. 仕様上の重要な決め（ここは実装前に固定推奨）

以下は仕様書として固定してください。そうしないと“運用が揺れて事故る”典型ポイントです。

1. **KILL はラッチ**（必ず止まり続ける）
2. **KILL 解除は現状プロトコル外**（ローカル操作/再起動）
3. **TTL は ESP32 受理時刻基準**（RPi時刻は不要）
4. **MODE を明示切替**（MODE_SET で切替）
5. **mode=AUTO 中は手動入力を無視**（KILL以外は無視）
