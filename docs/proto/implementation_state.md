# Protocol Implementation State

Goal: implement UART protocol per `docs/proto/overview.md` in ESP32 firmware.

Plan:
- [done] Step 1: protocol scaffolding (Protocol.h, CRC/COBS, packet reader/writer).
- [done] Step 2: control state + dispatcher + core handlers.
- [done] Step 3: main loop integration (UART poll, safety logic, BT/manual gating, ack/status).

Plan (current work):
- [done] Step 4: enforce AUTO_SETPOINT seq freshness + mark AUTO_INACTIVE on disallowed input.
- [done] Step 5: add STATUS transmit path (TxPort + rate limiting).
- [done] Step 6: wire STATUS fields from control loop (seq_applied, fault, speed/steer, age).

Notes:
- Track progress here; update after each step is completed.
