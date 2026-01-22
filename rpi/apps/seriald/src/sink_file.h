#pragma once
#include "async_logger.h"
#include <string>
#include <stdio.h>

class FileSink : public ILogSink {
public:
	explicit FileSink(const std::string& path);
	~FileSink();
	void write(const LogEvent& e) override;

private:
	std::string _path;
	FILE* _fp = nullptr;
};
