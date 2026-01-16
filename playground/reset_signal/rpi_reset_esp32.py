#!/usr/bin/env python3
import time
import gpiod

# ========= 設定 =========
# BCM番号（例：GPIO18）
RESET_GPIO = 18

# パルス幅（ENを確実に落とす）
PULSE_SEC = 0.15

# どのgpiochipか（通常は gpiochip0 でOK）
CHIP = "/dev/gpiochip0"
# =======================


def main():
    chip = gpiod.Chip(CHIP)
    line = chip.get_line(RESET_GPIO)

    # 初期状態は LOW（トランジスタOFF、ENを落とさない）
    line.request(consumer="esp32-reset", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])

    print(f"GPIO{RESET_GPIO} LOW (idle)")
    time.sleep(0.2)

    print(f"Pulse HIGH for {PULSE_SEC}s -> reset")
    line.set_value(1)  # トランジスタON -> ENをGNDへ -> reset
    time.sleep(PULSE_SEC)
    line.set_value(0)  # 戻す

    print("Done. Back to LOW (idle)")
    line.release()


if __name__ == "__main__":
    main()
