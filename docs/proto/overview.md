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
* **KILL**：緊急停止。**どのモードでも最優先**。解除は CLEAR_KILL のみ。

---

# 2. システム方針（最重要）

## 2.1 モードポリシー（確定）

* **自律中（AUTO_ACTIVE=true）**：

  * Bluetooth 手動入力は **無視**
  * 介入できるのは **KILL のみ**（BT でも UART でも可）
* **自律でない（AUTO_ACTIVE=false）**：

  * Bluetooth 手動入力を Drive に反映して良い
  * UART の AUTO コマンドを受けても適用しない（または受理するが AUTO_ACTIVE を上げない限り適用しない）

※この「自律中は手動無視」を成立させるため、**AUTO_ACTIVE を明示的に切り替えるメッセージ**をプロトコルに含めます（後述）。

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
* 対象：**header(VER〜LEN) + payload**
* CRCは little-endian 2byte を末尾に付与

---

## 3.3 パケット構造

### 3.3.1 Header v1（固定 6 bytes）

| フィールド |      型 | 説明                       |
| ------- | -----: | --------------------------- |
| `ver`   |     u8 | 現行は 1                     |
| `type`  |     u8 | メッセージ種別                |
| `flags` |     u8 | フラグ（共通）                |
| `seq`   |     u8 | シーケンス（0-255循環）        |
| `len`   | u16 LE | payload長（0..MAX_PAYLOAD）  |

**flags（共通）定義**

* bit0: `ACK_REQ`（1=ACKが欲しい）
* bit1: `RESERVED`
* bit2: `RESERVED`
* bit3..7: 予約

> MANUAL/AUTO は flags では表しません。理由：あなたの確定方針が「AUTO_ACTIVE時は手動無視」なので、入力の“出所”は ESP32 内部のソース（BT）と UART（RPi）で明確に分離でき、flags に混ぜると事故るためです。

---

## 3.4 type 定義（v1）

| type | 名前              | 方向      | payload  | 目的                             |
| ---: | ---------------- | --------- | -------- | -----------------------         |
| 0x01 | `AUTO_SETPOINT`  | RPi→ESP   | 固定      | 自律の操舵/速度/TTL               |
| 0x02 | `AUTO_MODE`      | RPi→ESP   | 固定      | 自律開始/停止（AUTO_ACTIVEの切替） |
| 0x03 | `HEARTBEAT`      | RPi→ESP   | なし      | 生存通知                         |
| 0x04 | `KILL`           | 双方向     | なし      | 即時停止（ラッチ）                |
| 0x05 | `CLEAR_KILL`     | 双方向     | なし      | KILL解除                        |
| 0x81 | `ACK`            | ESP→RPi   | 固定      | 受信確認（要求時）                |
| 0x82 | `STATUS`         | ESP→RPi   | 固定      | 状態通知（任意）                  |

---

## 3.5 payload 定義

### 3.5.1 `AUTO_MODE` (type=0x02) payload（2 bytes）

| フィールド    |  型 | 説明                          |
| -------- | -: | --------------------------- |
| `enable` | u8 | 1=AUTO_ACTIVE true, 0=false |
| `reason` | u8 | 予約（0）                       |

**挙動**

* enable=1：以後、UARTの `AUTO_SETPOINT` を適用対象とする。BT入力は無視。
* enable=0：以後、UARTの setpoint は受理しても適用しない（実装選択：破棄推奨）。BT入力が適用対象。

> 自律中に手動を受けないための「唯一の切替」です。RPiが制御権を握っていることを明示します。

---

### 3.5.2 `AUTO_SETPOINT` (type=0x01) payload（8 bytes）

物理量寄りで固定し、後からPIDや車両スケールが変わっても仕様が安定するようにします。

| フィールド      |  型 | 単位      | 説明                                                |
| ------------- | --: | -------- | --------------------------------------------------- |
| `steer_cdeg`  | i16 | 0.01°    | 目標舵角（0中心、左+右- などは実装で統一）                |
| `speed_cmd`   | i16 | “cmd単位” | 当面は -255..255 相当を推奨（現行 Engine に直結しやすい） |
| `ttl_ms`      | u16 | ms       | このコマンドの有効期限                                  |
| `distance_mm` | u16 | mm       | 将来拡張用（0なら無視）                                 |

**TTLの評価基準**

* ESP32 がこのフレームを **受理（CRC OK）した時刻 `t_rx_ms`** を起点に、

  * `now_ms - t_rx_ms > ttl_ms` なら失効 → STOP
* RPi時刻 `t_ms` は使わない（あなたの要望）

**distance_mm の扱い**

* v1では 0 以外でも “参照値”として保持して良い
* エンコーダ導入後に「距離到達で停止」へ拡張可能

---

### 3.5.3 `HEARTBEAT` (type=0x03) payload なし

* len=0
* 受理したら `last_hb_ms=now` を更新

---

### 3.5.4 `KILL` / `CLEAR_KILL` payload なし

* KILL受理：`kill_latched=true` にする（必ずラッチ）
* CLEAR_KILL受理：`kill_latched=false` にする（ただし、AUTO_ACTIVEやTTL等の停止条件が残っていれば停止のまま）

---

### 3.5.5 `ACK` (type=0x81) payload（4 bytes）

| フィールド    | 型 | 説明          |
| ----------- | -: | ------------ |
| `type_echo` | u8 | 受理した元type |
| `seq_echo`  | u8 | 受理した元seq  |
| `code`      | u8 | 結果コード     |
| `detail`    | u8 | 予約          |

**code 定義**

* 0: OK
* 1: CRC_FAIL（※CRC failは通常ACKしない。統計にしたければ別途）
* 2: BAD_VER
* 3: BAD_LEN
* 4: UNSUPPORTED_TYPE
* 5: NOT_ALLOWED（例：AUTO_ACTIVE中に特定操作を拒否、など拡張用）

---

### 3.5.6 `STATUS` (type=0x82) payload（12 bytes）

| フィールド         |  型 | 単位       | 説明                          |
| ---------------- | --: | -------- | ---------------------------------- |
| `seq_applied`    |  u8 | -        | 最後に適用した `AUTO_SETPOINT.seq`    |
| `auto_active`    |  u8 | -        | 1/0                                |
| `fault`          | u16 | bitfield | 故障/停止要因                        |
| `speed_now`      | i16 | cmd単位    | 現在速度（暫定は指令値でも良い）       |
| `steer_now_cdeg` | i16 | 0.01°    | 現在舵角（暫定は指令値でも良い）        |
| `age_ms`         | u16 | ms       | 最終AUTO_SETPOINT受理からの経過       |

**fault bitfield（v1案）**

* bit0: KILL_LATCHED
* bit1: HB_TIMEOUT
* bit2: TTL_EXPIRED
* bit3: AUTO_INACTIVE（AUTO_ACTIVE=falseなのにAUTO_SETPOINTが来ている等、運用監視）
* bit4..15: 予約

---

## 3.6 タイムアウト・周波数（推奨値）

* RPi→ESP32 `AUTO_SETPOINT`：50〜200Hz
* `ttl_ms`：制御周期の2〜3倍（例：送信100Hzなら ttl=30〜50ms）
* `HEARTBEAT`：10〜50Hz
* `hb_timeout_ms`（ESP32内部）：200ms（暫定）
* `STATUS`：20Hz（必要なときだけ）

---

# 4. ESP32 制御ロジック仕様（安全優先順位）

制御ループ（例：200Hz = 5ms周期）で必ずこの順で評価します。

1. **KILL**：`kill_latched==true` → `Engine.stop()` + `Steer.center()`（または安全角）
2. **AUTO_ACTIVE==true のとき**：

   * `hb_timeout`（HB使うなら）→ STOP
   * `ttl_expired` → STOP
   * 有効なら `AUTO_SETPOINT` を適用
3. **AUTO_ACTIVE==false のとき**：

   * Bluetooth入力を適用（ただし KILL 優先）

> これにより「自律中は手動無視（介入はKILLのみ）」がプロトコルレベルで崩れません。

---

# 5. ESP32 実装設計（拡張性＝type追加だけで機能追加）

あなたの現行ツリーを尊重し、最小の侵襲で移行できる形にします。

## 5.1 追加するコンポーネント（提案）

* `comm/Protocol.h`：ヘッダ構造体、type enum、payload struct
* `comm/CobsFramer.*`：0x00区切り受信、COBS decode/encode
* `comm/Crc16.*`：CRC16-CCITT
* `comm/Dispatcher.*`：type→handler 登録（関数ポインタ or インタフェース）
* `comm/handlers/*.cpp`：AUTO_SETPOINT/AUTO_MODE/HEARTBEAT/KILL/CLEAR_KILL
* `control/SafetyState.*`：kill_latched, auto_active, timers, fault
* `control/AutoCommandStore.*`：最新setpoint、受理時刻、seq 等
* `control/InputSource.*`（任意だが強く推奨）：BT/UART の入力を統一して Drive に入れる

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

## 6.1 Protocol 定義（例：`src/comm/Protocol.h`）

```cpp
#pragma once
#include <stdint.h>

namespace proto {

static constexpr uint8_t VER = 1;
static constexpr size_t MAX_PAYLOAD = 64; // 必要に応じて

enum class Type : uint8_t {
  AUTO_SETPOINT = 0x01,
  AUTO_MODE     = 0x02,
  HEARTBEAT     = 0x03,
  KILL          = 0x04,
  CLEAR_KILL    = 0x05,
  ACK           = 0x81,
  STATUS        = 0x82,
};

enum : uint8_t {
  FLAG_ACK_REQ = 1u << 0,
};

#pragma pack(push, 1)
struct Header {
  uint8_t  ver;
  uint8_t  type;
  uint8_t  flags;
  uint8_t  seq;
  uint16_t len;  // little-endian on wire
};

struct AutoModePayload {
  uint8_t enable; // 1 or 0
  uint8_t reason; // reserved
};

struct AutoSetpointPayload {
  int16_t  steer_cdeg;   // 0.01 deg
  int16_t  speed_cmd;    // -255..255 推奨（現行Engineに直結）
  uint16_t ttl_ms;
  uint16_t distance_mm;  // 0なら無視
};

struct AckPayload {
  uint8_t type_echo;
  uint8_t seq_echo;
  uint8_t code;
  uint8_t detail;
};

struct StatusPayload {
  uint8_t  seq_applied;
  uint8_t  auto_active;
  uint16_t fault;
  int16_t  speed_now;
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

## 6.3 AutoCommandStore（例：`src/control/AutoCommandStore.h`）

```cpp
#pragma once
#include <stdint.h>
#include "../comm/Protocol.h"

struct AutoCommandStore {
  bool     has = false;
  uint8_t  seq = 0;
  proto::AutoSetpointPayload sp{};
  uint32_t rx_ms = 0;

  void set(uint8_t s, const proto::AutoSetpointPayload& p, uint32_t now_ms) {
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

## 6.4 Dispatcher（例：`src/comm/Dispatcher.h`）

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

## 6.5 Context（例：`src/comm/Context.h`）

```cpp
#pragma once
#include "../hardware/Drive.h"
#include "../control/SafetyState.h"
#include "../control/AutoCommandStore.h"

struct TxPort {
  // ACK/STATUS を返したくなった時に実装
  void sendAck(uint8_t type_echo, uint8_t seq_echo, uint8_t code);
};

struct Context {
  Drive& drive;
  SafetyState& safety;
  AutoCommandStore& autoCmd;
  TxPort& tx;
};
```

## 6.6 ハンドラ例：AUTO_MODE（例：`src/comm/handlers/AutoModeHandler.cpp`）

```cpp
#include "../Protocol.h"
#include "../Dispatcher.h"
#include "../Context.h"
#include <string.h>

static inline const proto::Header* hdr(const FrameView& f) {
  return reinterpret_cast<const proto::Header*>(f.data);
}

bool handleAutoMode(const FrameView& f, Context& ctx) {
  const auto* h = hdr(f);
  if (h->len != sizeof(proto::AutoModePayload)) {
    ctx.tx.sendAck(h->type, h->seq, 3); // BAD_LEN
    return true;
  }
  proto::AutoModePayload p{};
  memcpy(&p, f.data + sizeof(proto::Header), sizeof(p));

  ctx.safety.auto_active = (p.enable != 0);
  ctx.tx.sendAck(h->type, h->seq, 0);
  return true;
}
```

## 6.7 ハンドラ例：AUTO_SETPOINT（例：`src/comm/handlers/AutoSetpointHandler.cpp`）

```cpp
#include "../Protocol.h"
#include "../Dispatcher.h"
#include "../Context.h"
#include "../../lib/common/Time.h"
#include <string.h>

static inline const proto::Header* hdr(const FrameView& f) {
  return reinterpret_cast<const proto::Header*>(f.data);
}

bool handleAutoSetpoint(const FrameView& f, Context& ctx) {
  const auto* h = hdr(f);
  if (h->len != sizeof(proto::AutoSetpointPayload)) {
    ctx.tx.sendAck(h->type, h->seq, 3); // BAD_LEN
    return true;
  }
  if (!ctx.safety.auto_active) {
    // 自律が有効でないのに来た：受理するか破棄するか。仕様としては破棄推奨。
    ctx.tx.sendAck(h->type, h->seq, 5); // NOT_ALLOWED
    return true;
  }

  proto::AutoSetpointPayload p{};
  memcpy(&p, f.data + sizeof(proto::Header), sizeof(p));

  const uint32_t now = mc::Time::ms();
  ctx.autoCmd.set(h->seq, p, now);

  ctx.tx.sendAck(h->type, h->seq, 0);
  return true;
}
```

## 6.8 ハンドラ例：KILL/CLEAR_KILL（例：`src/comm/handlers/KillHandler.cpp`）

```cpp
#include "../Protocol.h"
#include "../Dispatcher.h"
#include "../Context.h"

static inline const proto::Header* hdr(const FrameView& f) {
  return reinterpret_cast<const proto::Header*>(f.data);
}

bool handleKill(const FrameView& f, Context& ctx) {
  const auto* h = hdr(f);
  if (h->len != 0) {
    ctx.tx.sendAck(h->type, h->seq, 3);
    return true;
  }
  ctx.safety.kill_latched = true;
  ctx.tx.sendAck(h->type, h->seq, 0);
  return true;
}

bool handleClearKill(const FrameView& f, Context& ctx) {
  const auto* h = hdr(f);
  if (h->len != 0) {
    ctx.tx.sendAck(h->type, h->seq, 3);
    return true;
  }
  ctx.safety.kill_latched = false;
  ctx.tx.sendAck(h->type, h->seq, 0);
  return true;
}
```

## 6.9 main の制御ループ（現行 CSV 受信を置換する指針）

あなたの `main.cpp` は、次の3ブロックに整理します：

1. `uart_rx_poll()`：Serial2 から受信し、フレーム抽出→ディスパッチ
2. `bt_poll()`：Bluepad32 で pad 更新（ただし AUTO_ACTIVE なら適用しない。KILLボタンだけは許可）
3. `apply_control()`：安全優先順位で Drive に反映→ `drive.control()`

（COBSフレーマやCRC実装は長くなるのでここでは割愛しましたが、仕様は上記の通りです。）

---

# 7. Bluetooth（手動）入力の扱い仕様

## 7.1 自律中は手動無視（ただしKILLは許可）

* `AUTO_ACTIVE==true`：

  * `pad.update()` はしてもよい（接続維持・状態確認）
  * 走行コマンド（steer/speed）は **適用しない**
  * ただし **特定ボタン（例：HOME/START長押し）でKILLを発火**するのは許可（“介入はKILLのみ”の要件に一致）

## 7.2 自律でないときは手動操作OK

* `AUTO_ACTIVE==false`：

  * padから `Drive.setSpeed()`/`Drive.setAngle()` を更新して良い
  * `Drive` の timeout は手動でも更新される（安全停止）

---

# 8. 仕様上の重要な決め（ここは実装前に固定推奨）

以下は仕様書として固定してください。そうしないと“運用が揺れて事故る”典型ポイントです。

1. **KILL はラッチ**（必ず止まり続ける）
2. **KILL 解除は CLEAR_KILL のみ**（AUTO_SETPOINT では解除されない）
3. **TTL は ESP32 受理時刻基準**（RPi時刻は不要）
4. **AUTO_ACTIVE を明示切替**（AUTO_MODE enable/disable）
5. **AUTO_ACTIVE 中は手動入力を無視**（KILL以外は無視）
