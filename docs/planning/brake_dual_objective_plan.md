# 後退禁止・確実停止の両立 改修計画書

**作成日**: 2026-02-07  
**更新日**: 2026-02-07  
**ステータス**: 計画（単純化リファクタ）

---

## 1. 概要

「**前進のみ（後退させない）**」と「**強い減速（壁に突っ込まない）**」を両立しつつ、**ファイルごと削って全体を単純化**する。

### 1.1 目標アーキテクチャ

| モジュール | 役割 |
|------------|------|
| **SpeedController** | 前進 PWM のみ |
| **BrakeController** | 1個だけ（短い逆転パルス＋最終的にアクティブブレーキ） |
| **Tsd20Limiter** | 速度 cap ＋ stop_requested / stop_level だけ |

Speed / Decel / ABS / Brake が並列していた構造をやめ、**SpeedController → BrakeController の1段**に潰す。

### 1.2 現状の主因（衝突の構造バグ）

| 問題 | 説明 |
|------|------|
| 距離が近いほど弱ブレーキ | `BrakeController` の `ratio = d_mm/stop_dist` で duty 上限が縮む実装。近づいた瞬間に弱ブレーキ→衝突の原因 |
| 肥大化した並列制御 | Speed / Decel / ABS / Brake が同時に口を出す状態 |
| Decel が負 PWM を混ぜる | 減速＝負PWM＝連続後退の温床 |
| STALE でブレーキ解除 | Tsd20Limiter は STALE で `stop_requested=true` を出すが、BrakeController は STALE で return（ブレーキ無し）してしまう |

---

## 2. 削除対象（git rm）

### 削除するファイル

- `firmware/src/control/AbsController.h`
- `firmware/src/control/AbsController.cpp`
- `firmware/src/control/DecelController.h`
- `firmware/src/control/DecelController.cpp`

ABS と Decel を消して、**ブレーキは BrakeController 1個**に統合する。

---

## 3. 新しい「段階的逆転ブレーキ」の考え方

停止が必要（`stop_requested`）になったら BrakeController が以下を繰り返す：

1. **逆転パルス**：`-255` を **200ms だけ**出す（連続逆転は禁止）
2. **惰行**：`0` を **数十 ms**（例：60ms）出す
3. **再評価**：IMU の `v_est` がまだ大きいなら、もう1回だけパルス（最大 N 回で打ち止め）
4. 速度が落ちたら **アクティブブレーキ（短絡制動）**で止めきる（後退リスク 0）

### 安全策

| 条件 | 動作 |
|------|------|
| `v_est` が小さい（例：350mm/s 未満） | **逆転パルス禁止**（後退しない） |
| `stop_level == STALE` | **逆転せず最大ブレーキ**（フェイルセーフ） |

---

## 4. 具体的な差分（最小構成）

### 4.1 Config.h

ABS/DECEL パラメータを削除し、逆転パルス用だけ追加：

```diff
- static constexpr bool ABS_ENABLE                = true;
- static constexpr bool ABS_ENABLE_IN_MANUAL      = false;
- static constexpr bool ABS_REQUIRE_CALIB         = true;
- static constexpr int ABS_SPEED_MARGIN_MM_S      = 150;
- static constexpr uint16_t ABS_TSD_TRIGGER_MM    = 320;
- static constexpr int ABS_V_CMD_MAX_MM_S         = 200;
- static constexpr int ABS_V_EST_MIN_MM_S         = 300;
-
- static constexpr float DECEL_TAU_S             = 0.25f;
- static constexpr int   DECEL_V_ERR_DEADBAND_MM_S = 120;
- static constexpr int   DECEL_STOP_EPS_MM_S     = 120;
- static constexpr int   DECEL_PWM_MAX_BRAKE     = 200;
- static constexpr float DECEL_KP                = 0.010f;
- static constexpr float DECEL_KI                = 0.002f;
- static constexpr int   DECEL_A_MIN_MM_S2       = 1500;
- static constexpr int   DECEL_A_CAP_FLOOR_MM_S2 = 3000;
- static constexpr int   DECEL_A_CAP_MAX_MM_S2   = 12000;
-
  static constexpr int BRAKE_V_EPS_MM_S = 150;
  static constexpr int BRAKE_PWM_MAX = 80;
  static constexpr int BRAKE_PWM_MIN = 20;
  static constexpr uint32_t BRAKE_HOLD_MS = 120;
  static constexpr uint32_t BRAKE_RAMP_MS = 150;
  static constexpr uint32_t BRAKE_COOLDOWN_MS = 100;
+
+ // 逆転パルスブレーキ（連続逆転は禁止）
+ static constexpr int      BRAKE_REV_PWM = 255;          // 逆転PWM（符号はコード側で負にする）
+ static constexpr uint32_t BRAKE_REV_PULSE_MS = 200;     // 逆転する時間
+ static constexpr uint32_t BRAKE_REV_COAST_MS = 60;      // パルス間の惰行
+ static constexpr uint8_t  BRAKE_REV_MAX_PULSES = 4;     // 最大パルス回数
+ static constexpr int      BRAKE_REV_MIN_V_MM_S = 900;   // これ以上速いときだけ逆転を許可
+ static constexpr int      BRAKE_REV_EXIT_V_MM_S = 350;  // これ未満になったら逆転禁止
```

### 4.2 SafetySupervisor

Tsd20Limiter だけにする（ABS を完全撤去）：

**SafetySupervisor.h**

```diff
- #include "AbsController.h"
  #include "StopLevel.h"
  #include "Tsd20Limiter.h"
  ...
  struct SafetyDiag {
    Tsd20Diag tsd{};
-   AbsDiag abs{};
  };
  ...
  struct SafetyResult {
    Targets targets{};
-   bool brake_mode = false;
    bool stop_requested = false;
    StopLevel stop_level = StopLevel::NONE;
  };
  class SafetySupervisor {
  ...
  private:
    Tsd20Limiter _tsd;
-   AbsController _abs;
  };
```

**SafetySupervisor.cpp**

```diff
  SafetyResult SafetySupervisor::apply(...){
    ...
    out.targets.speed_mm_s =
      _tsd.limit(..., &d->tsd);
    out.stop_requested = diag ? diag->tsd.stop_requested : false;
    out.stop_level     = diag ? diag->tsd.stop_level     : StopLevel::NONE;
-   const bool imu_ok_for_abs = ...
-   bool abs_active = false;
-   out.targets.speed_mm_s = _abs.apply(..., &d->abs, &abs_active);
-   out.brake_mode = abs_active;
    return out;
  }
```

### 4.3 BrakeController

逆転パルス＋最終短絡制動へ統合（Decel 不要）。出力を「ブレーキ duty」だけでなく「PWM 上書き（逆転/惰行）」も出せるようにする。

**BrakeController.h**

```diff
  struct BrakeControllerOutput {
-   uint8_t brake_duty = 0;
-   bool active = false;
+   // PWMを上書きする場合（逆転パルス / 惰行0固定）
+   int16_t pwm_cmd = 0;
+   bool pwm_override = false;
+
+   // アクティブブレーキ（短絡制動）
+   uint8_t brake_duty = 0;
+   bool brake_active = false;
+
+   // engine rate_down をブレーキ用にするか
+   bool brake_mode = false;
  };
  class BrakeController {
  public:
-   BrakeControllerOutput update(bool stop_requested, StopLevel stop_level,
-                                uint16_t d_mm, float v_est_mm_s, uint32_t now_ms);
+   BrakeControllerOutput update(bool stop_requested, StopLevel stop_level,
+                                uint16_t d_mm, float v_est_mm_s,
+                                float a_brake_cap_mm_s2,
+                                uint32_t now_ms);
  private:
+   enum class Phase : uint8_t { IDLE=0, REV=1, COAST=2 };
+   Phase _phase = Phase::IDLE;
+   uint32_t _phase_until_ms = 0;
+   uint8_t _pulse_count = 0;
    uint32_t _stop_since_ms = 0;
    uint32_t _last_stop_ms = 0;
    float _brake_ramp = 0.0f;
+   void reset_();
  };
```

**BrakeController.cpp**（主要ロジック）

- `reset_()` で状態をクリア
- `stop_level == STALE` は逆転せず最大短絡制動
- 低速域（`v_est <= BRAKE_REV_EXIT_V_MM_S`）は逆転禁止
- Phase: REV → COAST → IDLE のサイクルで逆転パルス
- **「近いほど強く」**：`close = 1.0f - (d_mm/stop_dist)` で duty を決める（現状の逆を修正）

### 4.4 main.cpp

Decel/ABS 配線を撤去して BrakeController だけにする：

```diff
- #include "control/DecelController.h"
  ...
  static SafetySupervisor safety;
  static SpeedController speed_ctl;
- static DecelController decel_ctl;
  static BrakeController brake_ctl;
  ...
- static bool g_abs_active = false;
- static int16_t g_decel_diag_pwm = 0;
- ...（decel系diag一式）...
+ static bool g_brake_mode = false;
  ...
  static void applyTargets_(uint32_t now_ms, float dt_s) {
-   const bool abs_allowed = !g_state.killed && (cfg::ABS_ENABLE && ...);
    ...
    const SafetyResult safe = safety.apply(...);
-   g_abs_active = safe.brake_mode;
    ...
    const SpeedControlOutput out = speed_ctl.update(...);
-   const DecelControlOutput decel_out = decel_ctl.update(...);
-   const BrakeControllerOutput brake_out = brake_ctl.update(...);
+   const BrakeControllerOutput brake_out = brake_ctl.update(
+       safe.stop_requested, safe.stop_level, g_tsd_state.mm, v_est,
+       imu_state.a_brake_cap_mm_s2, now_ms);
+
+   g_brake_mode = brake_out.brake_mode || brake_out.brake_active || brake_out.pwm_override;
+
-   int16_t pwm_cmd = out.pwm_cmd;
-   if (decel_out.active)
-     pwm_cmd = (pwm_cmd < decel_out.pwm_cmd) ? pwm_cmd : decel_out.pwm_cmd;
-   drive.setBrakeMode(g_abs_active || brake_out.active || decel_out.active);
+   int16_t pwm_cmd = out.pwm_cmd;
+   if (brake_out.pwm_override) pwm_cmd = brake_out.pwm_cmd;
+   drive.setBrakeMode(g_brake_mode);
    drive.setTargetMmS(safe.targets.speed_mm_s, now_ms);
    drive.setTargetPwm(pwm_cmd, now_ms);
-   drive.setTargetBrake(brake_out.brake_duty, brake_out.active, now_ms);
+   drive.setTargetBrake(brake_out.brake_duty, brake_out.brake_active, now_ms);
    ...
  }
  ...
  static void sendImuStatus_(uint32_t now_ms) {
    ...
-   if (g_abs_active) flags |= 1u << 2;
+   if (g_brake_mode) flags |= 1u << 2;
  }
```

---

## 5. 簡素化で得られるポイント

| ポイント | 効果 |
|----------|------|
| 制御の分岐が 1 段 | SpeedController → BrakeController のみ（「Speed/Decel/ABS/Brake が同時に口を出す」状態を根絶） |
| 逆転は 200ms パルス以外に存在しない | 「逆転し続けて後退」問題が構造的に起きない |
| 低速域は逆転禁止 | 停止間際の「行ったり来たり」も消える |
| 距離が近いほどブレーキが強くなる | 「間に合わず正面衝突」の確率が下がる |

---

## 6. 実行手順

```sh
git rm firmware/src/control/AbsController.h firmware/src/control/AbsController.cpp
git rm firmware/src/control/DecelController.h firmware/src/control/DecelController.cpp

# 上記 diff どおりに以下を編集
# - Config.h
# - SafetySupervisor.h / SafetySupervisor.cpp
# - BrakeController.h / BrakeController.cpp
# - main.cpp
```

---

## 7. 将来の拡張（オプション）

**逆転パルスの「効き」を IMU の decel 実測で自動調整**（パルス回数/長さを増減）も、追加ロジックは数十行で済む。DecelController の復活は不要。

---

## 8. 参照

- `docs/planning/decel_controller_status.md`：DecelController の現状（削除予定）
- `docs/planning/abs_backward_investigation.md`：後退問題の調査
