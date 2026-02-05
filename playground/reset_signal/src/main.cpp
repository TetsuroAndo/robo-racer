#include "esp_system.h"
#include <Arduino.h>

RTC_DATA_ATTR uint32_t bootCount = 0;

static const char *resetReasonToStr(esp_reset_reason_t r) {
	switch (r) {
	case ESP_RST_POWERON:
		return "POWERON";
	case ESP_RST_EXT:
		return "EXT";
	case ESP_RST_SW:
		return "SW";
	case ESP_RST_BROWNOUT:
		return "BROWNOUT";
	default:
		return "OTHER";
	}
}

void setup() {
	Serial.begin(115200);
	delay(200);

	bootCount++;
	auto reason = esp_reset_reason();

	Serial.println();
	Serial.println("*** PLAYGROUND RESET-SIGNAL FW BOOTED ***");
	Serial.printf("bootCount=%lu\n", (unsigned long)bootCount);
	Serial.printf("resetReason=%s (%d)\n", resetReasonToStr(reason),
				  (int)reason);
}

void loop() {
	delay(1000);
	Serial.println("[alive]");
}
