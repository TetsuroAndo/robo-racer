#pragma once
#include "async_logger.h"

class StdoutSink : public ILogSink {
public:
	void write(const LogEvent& e) override;
};
