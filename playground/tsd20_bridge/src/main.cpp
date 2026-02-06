#include <Arduino.h>
#include <Wire.h>

// =======================
// Pin Assign (EDIT HERE)
// =======================
// TSD20 shared pins (same wires used for UART switching and I2C operation)
static const int PIN_TSD20_SDA = 32; // I2C SDA after switch
static const int PIN_TSD20_SCL = 33; // I2C SCL after switch

// Raspberry Pi <-> ESP32 UART (control/log)
static const int PIN_RPI_UART_RX = 16; // ESP32 RX (from RPi TX)
static const int PIN_RPI_UART_TX = 17; // ESP32 TX (to   RPi RX)

// =======================
// Config
// =======================
static const uint32_t I2C_FREQ_HZ = 100000;
static const uint8_t TSD20_ADDR = 0x52;

static const uint8_t REG_DIST_H = 0x00;
static const uint8_t REG_LASER_CTRL = 0x02;
static const uint8_t REG_ID = 0x03;
static const uint8_t EXPECTED_ID = 0x4A;

static const uint32_t RPI_BAUD = 115200;
static const uint32_t TSD20_UART_BAUD_PRIMARY = 460800;
static const uint32_t TSD20_UART_BAUD_FALLBACK = 115200;

// NOTE: Avoid name "PI" (Arduino defines PI macro)
static HardwareSerial RpiSerial(1);
static HardwareSerial TsdUart(2);

// =======================
// I2C helpers
// =======================
static bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
	// repeated-start first
	Wire.beginTransmission(addr);
	Wire.write(reg);
	uint8_t rc = Wire.endTransmission(false);
	if (rc == 0) {
		size_t got = Wire.requestFrom((int)addr, (int)len, (int)true);
		if (got == len) {
			for (size_t i = 0; i < len; i++)
				buf[i] = (uint8_t)Wire.read();
			return true;
		}
	}

	// fallback STOP
	Wire.beginTransmission(addr);
	Wire.write(reg);
	rc = Wire.endTransmission(true);
	if (rc != 0)
		return false;

	size_t got = Wire.requestFrom((int)addr, (int)len, (int)true);
	if (got != len)
		return false;

	for (size_t i = 0; i < len; i++)
		buf[i] = (uint8_t)Wire.read();
	return true;
}

static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t v) {
	Wire.beginTransmission(addr);
	Wire.write(reg);
	Wire.write(v);
	return (Wire.endTransmission(true) == 0);
}

static bool tsd20ProbeI2C(uint8_t &id) {
	return i2cReadReg(TSD20_ADDR, REG_ID, &id, 1);
}

static bool tsd20ReadDistanceMm(uint16_t &mm) {
	uint8_t b[2];
	if (!i2cReadReg(TSD20_ADDR, REG_DIST_H, b, 2))
		return false;
	mm = (uint16_t(b[0]) << 8) | b[1];
	return true;
}

// =======================
// UART -> I2C switching (Change IIC command)
// =======================
static bool tsd20SendChangeIICAtBaud(uint32_t baud, int rxPin, int txPin,
									 bool &sawResponse) {
	sawResponse = false;
	// Change IIC: 5A 1F 02 1F 1F A0  (manual)
	const uint8_t cmd[] = {0x5A, 0x1F, 0x02, 0x1F, 0x1F, 0xA0};

	// Use same wires: RX must be on pin connected to TSD20 TX (Pin3),
	// TX must be on pin connected to TSD20 RX (Pin4).
	Wire.end();
	delay(10);
	TsdUart.end();
	delay(20);
	TsdUart.begin(baud, SERIAL_8N1, /*rx=*/rxPin, /*tx=*/txPin);
	delay(50);

	// Flush any garbage
	while (TsdUart.available())
		(void)TsdUart.read();

	TsdUart.write(cmd, sizeof(cmd));
	TsdUart.flush();

	// Read optional response for ~200ms (some units respond, some just switch)
	uint32_t t0 = millis();
	uint8_t buf[16];
	size_t n = 0;
	while (millis() - t0 < 200) {
		while (TsdUart.available() && n < sizeof(buf)) {
			buf[n++] = (uint8_t)TsdUart.read();
		}
	}

	TsdUart.end();
	delay(30);

	// If we saw the expected response header, treat as success; otherwise still
	// try I2C probe. Expected response (manual): 5A 9F 02 1F 1F 20
	if (n >= 6) {
		for (size_t i = 0; i + 5 < n; i++) {
			if (buf[i] == 0x5A && buf[i + 1] == 0x9F && buf[i + 2] == 0x02) {
				sawResponse = true;
				return true;
			}
		}
	}
	return false; // unknown; caller will re-probe I2C anyway
}

static bool tsd20SendChangeIIC() {
	bool sawResponse = false;
	(void)tsd20SendChangeIICAtBaud(TSD20_UART_BAUD_PRIMARY,
								   /*rx=*/PIN_TSD20_SCL,
								   /*tx=*/PIN_TSD20_SDA, sawResponse);
	if (!sawResponse) {
		Serial.printf("[TSD20] no UART response @%lu. Trying %lu...\n",
					  (unsigned long)TSD20_UART_BAUD_PRIMARY,
					  (unsigned long)TSD20_UART_BAUD_FALLBACK);
		(void)tsd20SendChangeIICAtBaud(TSD20_UART_BAUD_FALLBACK,
									   /*rx=*/PIN_TSD20_SCL,
									   /*tx=*/PIN_TSD20_SDA, sawResponse);
	}
	if (!sawResponse) {
		Serial.println("[TSD20] retry UART with swapped pins...");
		(void)tsd20SendChangeIICAtBaud(TSD20_UART_BAUD_PRIMARY,
									   /*rx=*/PIN_TSD20_SDA,
									   /*tx=*/PIN_TSD20_SCL, sawResponse);
	}
	if (!sawResponse) {
		Serial.printf(
			"[TSD20] no UART response @%lu (swapped). Trying %lu...\n",
			(unsigned long)TSD20_UART_BAUD_PRIMARY,
			(unsigned long)TSD20_UART_BAUD_FALLBACK);
		(void)tsd20SendChangeIICAtBaud(TSD20_UART_BAUD_FALLBACK,
									   /*rx=*/PIN_TSD20_SDA,
									   /*tx=*/PIN_TSD20_SCL, sawResponse);
	}
	return sawResponse;
}

// =======================
// UI / Pi logging
// =======================
static void piStatus(const char *msg) {
	RpiSerial.printf("S,%s,%lu\n", msg, (unsigned long)millis());
}
static void piDistance(uint16_t mm) {
	RpiSerial.printf("D,%u,%lu\n", mm, (unsigned long)millis());
}

static bool g_i2cSwapped = false;
static bool g_i2cKnown = false;

static void printBanner() {
	Serial.println("=== TSD20 AutoSwitch(I2C) -> RPi(UART) Bridge ===");
	Serial.printf("TSD20 pins (cfg): SDA=GPIO%d, SCL=GPIO%d\n", PIN_TSD20_SDA,
				  PIN_TSD20_SCL);
	if (g_i2cKnown) {
		const int sdaPin = g_i2cSwapped ? PIN_TSD20_SCL : PIN_TSD20_SDA;
		const int sclPin = g_i2cSwapped ? PIN_TSD20_SDA : PIN_TSD20_SCL;
		Serial.printf("TSD20 pins (active): SDA=GPIO%d, SCL=GPIO%d%s\n", sdaPin,
					  sclPin, g_i2cSwapped ? " (swapped)" : "");
	} else {
		Serial.println("TSD20 pins (active): unknown (auto-swap enabled)");
	}
	Serial.printf("RPi UART : RX=GPIO%d TX=GPIO%d baud=%lu\n", PIN_RPI_UART_RX,
				  PIN_RPI_UART_TX, (unsigned long)RPI_BAUD);
	Serial.println("Commands (USB Serial):");
	Serial.println("  i : send Change IIC now, then re-probe I2C");
	Serial.println("  p : probe I2C (read ID)");
	Serial.println("  o : laser ON (I2C)");
	Serial.println("  f : laser OFF (I2C)");
	Serial.println("  s : I2C scan");
	Serial.println("  h : help");
}

// =======================
// State
// =======================
static bool g_tsd20Ready = false;

static void initI2CWithPins(int sdaPin, int sclPin) {
	Wire.end();
	delay(10);
	Wire.begin(sdaPin, sclPin, I2C_FREQ_HZ);
	delay(30);
}

static void initI2C() {
	const int sdaPin = g_i2cSwapped ? PIN_TSD20_SCL : PIN_TSD20_SDA;
	const int sclPin = g_i2cSwapped ? PIN_TSD20_SDA : PIN_TSD20_SCL;
	initI2CWithPins(sdaPin, sclPin);
}

static bool probeI2CWithPins(int sdaPin, int sclPin, uint8_t &id) {
	initI2CWithPins(sdaPin, sclPin);
	return tsd20ProbeI2C(id);
}

static void i2cScan() {
	const int sdaPrimary = PIN_TSD20_SDA;
	const int sclPrimary = PIN_TSD20_SCL;
	const int sdaSwap = PIN_TSD20_SCL;
	const int sclSwap = PIN_TSD20_SDA;

	g_i2cKnown = false;
	g_i2cSwapped = false;
	Serial.println("[I2C] scan start");
	uint8_t found = 0;
	initI2CWithPins(sdaPrimary, sclPrimary);
	for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
		Wire.beginTransmission(addr);
		uint8_t rc = Wire.endTransmission(true);
		if (rc == 0) {
			Serial.printf("[I2C] found addr=0x%02X (primary)\n", addr);
			found++;
		}
	}
	if (found == 0) {
		initI2CWithPins(sdaSwap, sclSwap);
		for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
			Wire.beginTransmission(addr);
			uint8_t rc = Wire.endTransmission(true);
			if (rc == 0) {
				Serial.printf("[I2C] found addr=0x%02X (swapped)\n", addr);
				found++;
			}
		}
		if (found > 0) {
			g_i2cSwapped = true;
			g_i2cKnown = true;
		}
	} else {
		g_i2cSwapped = false;
		g_i2cKnown = true;
	}
	Serial.printf("[I2C] scan done. found=%u\n", found);
}

static bool detectI2C(uint8_t &id) {
	g_i2cKnown = false;
	g_i2cSwapped = false;
	if (probeI2CWithPins(PIN_TSD20_SDA, PIN_TSD20_SCL, id)) {
		g_i2cSwapped = false;
		g_i2cKnown = true;
		return true;
	}
	if (probeI2CWithPins(PIN_TSD20_SCL, PIN_TSD20_SDA, id)) {
		g_i2cSwapped = true;
		g_i2cKnown = true;
		return true;
	}
	return false;
}

static void probeOrSwitch() {
	uint8_t id = 0;
	if (detectI2C(id)) {
		Serial.printf("[TSD20] I2C detected. ID=0x%02X\n", id);
		if (g_i2cSwapped)
			Serial.println("[TSD20] I2C pins are swapped (auto-corrected).");
		g_tsd20Ready = true;
		piStatus("tsd20_i2c_ok");
		return;
	}

	Serial.println(
		"[TSD20] NOT FOUND on I2C. Trying UART->I2C switch (Change IIC)...");
	piStatus("tsd20_i2c_fail_try_switch");

	(void)tsd20SendChangeIIC();

	// After switching, some units require power cycle. We still try immediate
	// re-probe.
	if (detectI2C(id)) {
		Serial.printf("[TSD20] I2C detected after switch. ID=0x%02X\n", id);
		if (g_i2cSwapped)
			Serial.println("[TSD20] I2C pins are swapped (auto-corrected).");
		g_tsd20Ready = true;
		piStatus("tsd20_i2c_ok_after_switch");
		return;
	}

	Serial.println("[TSD20] Still not found. Power-cycle TSD20 (OFF/ON) and "
				   "reboot ESP32, then retry.");
	g_tsd20Ready = false;
	piStatus("tsd20_need_power_cycle");
}

void setup() {
	Serial.begin(115200);
	delay(200);

	RpiSerial.begin(RPI_BAUD, SERIAL_8N1, PIN_RPI_UART_RX, PIN_RPI_UART_TX);
	delay(30);
	piStatus("boot");

	// Idle check
	pinMode(PIN_TSD20_SDA, INPUT_PULLUP);
	pinMode(PIN_TSD20_SCL, INPUT_PULLUP);
	delay(10);
	Serial.printf("[I2C] idle pins: SDA=%d SCL=%d\n",
				  digitalRead(PIN_TSD20_SDA), digitalRead(PIN_TSD20_SCL));

	printBanner();
	probeOrSwitch();

	if (g_tsd20Ready) {
		// Laser ON
		if (i2cWrite8(TSD20_ADDR, REG_LASER_CTRL, 0x01)) {
			Serial.println("[TSD20] laser ON");
			piStatus("laser_on");
		} else {
			Serial.println("[TSD20] laser ON failed");
			piStatus("laser_on_failed");
		}
	}
}

void loop() {
	// USB serial commands
	if (Serial.available()) {
		char c = (char)Serial.read();
		if (c == 'h' || c == '?')
			printBanner();
		else if (c == 'p') {
			uint8_t id = 0;
			initI2C();
			bool ok = tsd20ProbeI2C(id);
			Serial.printf("[PROBE] %s ID=0x%02X\n", ok ? "OK" : "FAIL", id);
		} else if (c == 'i') {
			Serial.println("[CMD] Change IIC + re-probe");
			probeOrSwitch();
		} else if (c == 'o') {
			initI2C();
			Serial.println(i2cWrite8(TSD20_ADDR, REG_LASER_CTRL, 0x01)
							   ? "[TSD20] laser ON"
							   : "[TSD20] laser ON failed");
		} else if (c == 'f') {
			initI2C();
			Serial.println(i2cWrite8(TSD20_ADDR, REG_LASER_CTRL, 0x00)
							   ? "[TSD20] laser OFF"
							   : "[TSD20] laser OFF failed");
		} else if (c == 's') {
			i2cScan();
		}
	}

	// periodic read
	static uint32_t lastMs = 0;
	const uint32_t intervalMs = 50;
	uint32_t now = millis();
	if (now - lastMs >= intervalMs) {
		lastMs = now;

		if (!g_tsd20Ready)
			return;

		initI2C();
		uint16_t mm = 0;
		if (tsd20ReadDistanceMm(mm)) {
			Serial.printf("[DIST] %u mm\n", mm);
			piDistance(mm);
		} else {
			Serial.println("[DIST] read failed");
			piStatus("dist_read_failed");
		}
	}
}
