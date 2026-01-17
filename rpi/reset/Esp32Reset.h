#pragma once

#include <chrono>
#include <string>

class Esp32Reset {
public:
    struct Config {
        std::string chip_path = "/dev/gpiochip0";
        unsigned int bcm_gpio = 18;
        bool assert_is_high = true;
        std::chrono::milliseconds pulse{200};
        std::chrono::milliseconds settle{100};
    };

    explicit Esp32Reset(Config cfg = {});
    void pulse();

private:
    Config cfg_;
};
