#!/usr/bin/env python3
import time

import gpiod
from gpiod.line import Direction, Value


def pulse_reset(
    chip_path: str = "/dev/gpiochip0",
    bcm_gpio: int = 18,
    pulse_sec: float = 0.2,
    assert_is_high: bool = True,
) -> None:
    inactive = Value.INACTIVE if assert_is_high else Value.ACTIVE
    active = Value.ACTIVE if assert_is_high else Value.INACTIVE

    settings = gpiod.LineSettings(
        direction=Direction.OUTPUT,
        active_low=False,
        output_value=inactive,
    )

    with gpiod.request_lines(
        chip_path,
        consumer="rpi-esp32-reset",
        config={bcm_gpio: settings},
    ) as req:
        req.set_value(bcm_gpio, active)
        time.sleep(pulse_sec)
        req.set_value(bcm_gpio, inactive)
