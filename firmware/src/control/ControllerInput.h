#pragma once
#include <Arduino.h>
#include <Bluepad32.h>

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
