#include "ControllerInput.h"

ControllerPtr ControllerInput::myCtl_ = nullptr;

void ControllerInput::onConnected(ControllerPtr ctl) {
  Serial.println("Controller connected");
  if (!myCtl_) {
    myCtl_ = ctl;
    Serial.println("Using this controller as primary.");
  }
}

void ControllerInput::onDisconnected(ControllerPtr ctl) {
  Serial.println("Controller disconnected");
  if (myCtl_ == ctl) {
    myCtl_ = nullptr;
    Serial.println("Primary controller cleared.");
  }
}

void ControllerInput::begin() {
  BP32.setup(&ControllerInput::onConnected, &ControllerInput::onDisconnected);
}

void ControllerInput::update() {
  BP32.update();

  toggleRunningEdge_ = false;

  if (!myCtl_ || !myCtl_->isConnected()) {
    return;
  }

  // '-' ボタンのエッジ検出（押した瞬間）
  bool selectNow = myCtl_->miscSelect();
  if (selectNow && !prevSelect_) {
    toggleRunningEdge_ = true;
  }
  prevSelect_ = selectNow;

  readPad_();
}

bool ControllerInput::consumeToggleRunning() {
  if (toggleRunningEdge_) {
    toggleRunningEdge_ = false;
    return true;
  }
  return false;
}

bool ControllerInput::isConnected() const {
  return (myCtl_ && myCtl_->isConnected());
}

void ControllerInput::readPad_() {
  st_.lt = myCtl_->brake();
  st_.rt = myCtl_->throttle();

  // Proコン見た目に合わせて A↔B / X↔Y
  st_.A = myCtl_->b();
  st_.B = myCtl_->a();
  st_.X = myCtl_->y();
  st_.Y = myCtl_->x();

  st_.select = myCtl_->miscSelect();
  st_.start  = myCtl_->miscStart();
  st_.home   = myCtl_->miscHome();

  st_.dpad = myCtl_->dpad();
}
