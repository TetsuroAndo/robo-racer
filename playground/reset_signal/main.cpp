#include "esp_system.h" // esp_reset_reason()
#include <Arduino.h>

// DeepSleep以外のリセットでも保持される(多くのケースで) RTC領域
RTC_DATA_ATTR uint32_t bootCount = 0;

static const char *resetReasonToStr(esp_reset_reason_t r) {
	switch (r) {
	case ESP_RST_POWERON:
		return "POWERON";
	case ESP_RST_EXT:
		return "EXT (external reset)";
	case ESP_RST_SW:
		return "SW (software reset)";
	case ESP_RST_PANIC:
		return "PANIC";
	case ESP_RST_INT_WDT:
		return "INT_WDT";
	case ESP_RST_TASK_WDT:
		return "TASK_WDT";
	case ESP_RST_WDT:
		return "WDT";
	case ESP_RST_DEEPSLEEP:
		return "DEEPSLEEP";
	case ESP_RST_BROWNOUT:
		return "BROWNOUT";
	case ESP_RST_SDIO:
		return "SDIO";
	default:
		return "UNKNOWN";
	}
}

void setup() {
	Serial.begin(115200);
	delay(200);

	bootCount++;

	esp_reset_reason_t reason = esp_reset_reason();

	Serial.println();
	Serial.println("=== ESP32 Playground Boot ===");
	Serial.printf("Boot count: %lu\n", (unsigned long)bootCount);
	Serial.printf("Reset reason: %s (%d)\n", resetReasonToStr(reason),
				  (int)reason);
	Serial.println("Waiting... (Toggle RPi reset to see this again)");
}

void loop() {
	// 生きていることが分かるようにハートビート
	static uint32_t last = 0;
	if (millis() - last > 1000) {
		last = millis();
		Serial.printf("[alive] uptime=%lu ms\n", (unsigned long)millis());
	}
}
