#!/usr/bin/env python3
import time
import gpiod
from gpiod.line import Direction, Value

CHIP_PATH = "/dev/gpiochip0"
RESET_GPIO = 18  # BCM番号
PULSE_SEC = 0.2


def main():
    settings = gpiod.LineSettings(
        direction=Direction.OUTPUT,
        active_low=False,  # “GPIOの電圧”をそのまま扱う
        output_value=Value.INACTIVE,  # 起動時はLOW（=リセットしない）
    )

    with gpiod.request_lines(
        CHIP_PATH,
        consumer="rpi-reset-esp32",
        config={RESET_GPIO: settings},
    ) as req:
        # RPi GPIO を HIGH にしてリセット（反転回路を前提）
        req.set_value(RESET_GPIO, Value.ACTIVE)  # HIGH
        time.sleep(PULSE_SEC)
        req.set_value(RESET_GPIO, Value.INACTIVE)  # LOW


if __name__ == "__main__":
    main()
