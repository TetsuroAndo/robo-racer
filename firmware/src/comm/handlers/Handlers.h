#pragma once

#include "../Dispatcher.h"

struct Context;

void registerHandlers(Dispatcher &dispatcher);

bool handleAutoMode(const proto::FrameView &frame, Context &ctx);
bool handleAutoSetpoint(const proto::FrameView &frame, Context &ctx);
bool handleHeartbeat(const proto::FrameView &frame, Context &ctx);
bool handleKill(const proto::FrameView &frame, Context &ctx);
bool handleClearKill(const proto::FrameView &frame, Context &ctx);
