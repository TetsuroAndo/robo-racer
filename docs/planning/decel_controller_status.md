# DecelController 実装ステータス

**注意**: 本モジュールは **削除予定**。`docs/planning/brake_dual_objective_plan.md` の単純化リファクタで BrakeController に統合される。

## 概要

速度目標が下がった際に能動制動（負PWM）を出し、停止距離を物理的に詰めるための DecelController を追加した。

## 背景

- 現状FWは「前進コマンド中は PWM を負にしない（pwm_lo=0）」ため、速度目標を 0 に落としても逆トルク（ブレーキ）を出せず、減速は転がり抵抗頼み
- `cmd_speed_mm_s` が 0 になった時点で `tsd_mm` が 1.2〜2.0m 残っていても、`a_long_mm_s2` がほぼ制動にならず、停止距離不足で衝突していた

## 実装方針

- **SpeedController**：前進速度を作る（負PWM禁止のまま）
- **DecelController**：減速・停止を作る（負PWMを限定的に許可、停止付近で抜く）

## 切替条件

| 条件 | 動作 |
|------|------|
| `v_est > v_target + deadband` | DecelController 有効（負PWM出力） |
| `v_est <= DECEL_STOP_EPS_MM_S` | ブレーキ解除（後退抑止） |
| それ以外 | SpeedController のみ（前進PWM） |

## 実装済み

- [x] `DecelController.h` / `DecelController.cpp` 追加
- [x] `Config.h` に DECEL_* パラメータ追加
- [x] `main.cpp` で PWM 合成（減速時は負PWMで上書き）
- [x] `drive.setBrakeMode(safe.brake_mode || decel_out.active)` で減速時のスルーレート高速化
- [x] `decel_ctl` ログ出力（active 時のみ）

## チューニングパラメータ（Config.h）

| パラメータ | 値 | 説明 |
|------------|-----|------|
| DECEL_TAU_S | 0.25 | v_err/tau → a_tgt |
| DECEL_V_ERR_DEADBAND_MM_S | 120 | 減速不要とみなす速度誤差 |
| DECEL_STOP_EPS_MM_S | 120 | これ以下でブレーキ解除 |
| DECEL_PWM_MAX_BRAKE | 200 | 制動PWMの上限 |
| DECEL_KP / DECEL_KI | 0.01 / 0.002 | 加速度誤差フィードバック |
| DECEL_A_MIN_MM_S2 | 1500 | 目標減速度の下限 |
| DECEL_A_CAP_FLOOR/MAX | 3000 / 12000 | a_brake_cap のクランプ範囲 |

## テスト観点

1. **停止指令ステップ**：`cmd_speed` 5000→0 で `a_long_lpf` が負側に入り、`v_est` が 0 へ収束すること
2. **停止距離**：`tsd_mm` が十分あるタイミングで 0 指令を出した場合、`tsd_min` が MARGIN を割らないこと
3. **後退抑止**：停止後に `pwm_cmd` が 0 へ戻り、`v_est` が負側へ行かないこと
4. **既存安全系の優先**：kill / TTL / TSD20 clamp / ABS が従来通り動作すること

## 関連改修（2026-02-07）

DecelController の負 PWM は **ブレーキ duty に変換**され、通常経路では後退しない形に変更済み。

**2026-02-07 更新**: 単純化リファクタにより DecelController は **削除**され、BrakeController に統合された。詳細は `docs/planning/brake_dual_objective_plan.md` を参照。
