#include "sink_file.h"
#include <stdio.h>

static const char *lvStr(LogLevel lv) {
	switch (lv) {
	case LogLevel::TRACE:
		return "TRACE";
	case LogLevel::DEBUG:
		return "DEBUG";
	case LogLevel::INFO:
		return "INFO";
	case LogLevel::WARN:
		return "WARN";
	case LogLevel::ERROR:
		return "ERROR";
	case LogLevel::FATAL:
		return "FATAL";
	}
	return "UNK";
}

FileSink::FileSink(const std::string &path) : _path(path) {
	_fp = fopen(_path.c_str(), "a");
}

FileSink::~FileSink() {
	if (_fp)
		fclose(_fp);
}

void FileSink::write(const LogEvent &e) {
	if (!_fp)
		return;
	fprintf(_fp, "[%s] %s\n", lvStr(e.level), e.text.c_str());
	fflush(_fp);
}
