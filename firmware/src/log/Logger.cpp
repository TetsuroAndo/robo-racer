#include "Logger.h"

void Logger::begin(uint32_t baud) {
  if (!started_) {
    Serial.begin(baud);
    delay(200);
    started_ = true;
  }
}

void Logger::printLine(
  bool connected,
  bool running,
  const PadState& st,
  int speed,
  const char* steer
) {
  // \r で行頭に戻る → 1行上書き
  Serial.printf(
    "\r[BP32:%s] [%s] "
    "A:%d B:%d "
    "DPAD:0x%02X "
    "SPD:%3d "
    "STR:%-6s ",
    connected ? "OK " : "NG ",
    running   ? "RUN" : "PAUSE",
    st.A, st.B,
    st.dpad,
    speed,
    steer
  );
}
