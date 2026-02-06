#pragma once
#include <Arduino.h>
#include "../config/Config.h"

struct PadState {
  int lt = 0;
  int rt = 0;

  bool A = false;
  bool B = false;
  bool X = false;
  bool Y = false;

  bool select = false; // -
  bool start  = false; // +
  bool home   = false; // HOME

  int dpad = 0;
};

#if MC_ENABLE_MANUAL
#include <Bluepad32.h>

class ControllerInput {
public:
  void begin();
  void update();

  bool isConnected() const;
  const PadState& state() const { return st_; }

  // Pause/Resume のトグルが起きた瞬間だけ true
  bool consumeToggleRunning();

private:
  static void onConnected(ControllerPtr ctl);
  static void onDisconnected(ControllerPtr ctl);

  static ControllerPtr myCtl_;
  PadState st_;

  bool prevSelect_ = false;
  bool toggleRunningEdge_ = false;

  void readPad_();
};
#else
class ControllerInput {
public:
  void begin() {}
  void update() {}

  bool isConnected() const { return false; }
  const PadState& state() const { return st_; }

  bool consumeToggleRunning() { return false; }

private:
  PadState st_{};
};
#endif
