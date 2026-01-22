# RPi Protocol Implementation State

Goal: implement UART protocol per `docs/proto/overview.md` on the RPi side.

Plan:
- [done] Step 1: add protocol codec (Protocol structs, CRC16, COBS, packet reader/writer).
- [done] Step 2: update UART open helpers for read/write + 921600 baud.
- [done] Step 3: replace CSV sender with protocol sender (AUTO_MODE, AUTO_SETPOINT, HEARTBEAT).
- [done] Step 4: add RX handling for ACK/STATUS (optional logging/state).

Notes:
- Track progress here; update after each step is completed.
