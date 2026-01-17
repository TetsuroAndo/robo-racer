#!/usr/bin/env python3
try:
    from src.reset.esp32_reset import pulse_reset
except ModuleNotFoundError:
    from reset.esp32_reset import pulse_reset


def main() -> None:
    pulse_reset(bcm_gpio=18, pulse_sec=0.2, assert_is_high=True)
    print("OK: ESP32 reset pulsed")


if __name__ == "__main__":
    main()
