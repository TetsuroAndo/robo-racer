#pragma once
#include <Arduino.h>
#include "comm/BleUart.h"
#include "comm/CommandMux.h"
#include "config/Config.h"
#include "control/ActuatorBus.h"
#include "hils/Hils.h"
#include "log/LogSinks.h"

namespace mc {

class App {
 public:
  App();

  void begin();
  void loop();

 private:
  BleUart _ble{};
  comm::CommandMux _commands;
  ActuatorBus _actuators;

  log::StreamSink _serialLogSink;
  log::StreamSink _bleLogSink;
  log::ProtocolSink _uartProtoLogSink;
  log::UdpSink _udpLogSink;
  hils::StreamSink _hilsSerialSink;
  hils::StreamSink _hilsBleSink;
  hils::Mux _hilsMux;

  comm::RcCommand _cmd{};
  uint32_t _lastCmdMs = 0;
  bool _timeoutActive = false;

  void configureLogging();
  void configureHils();
  bool readInputs(uint32_t nowMs);
  void applyOutputs(const comm::RcCommand &cmd, uint32_t nowMs);
};

} // namespace mc
