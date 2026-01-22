#include "sink_stdout.h"
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

void StdoutSink::write(const LogEvent &e) {
	fprintf(stdout, "[%s] %s\n", lvStr(e.level), e.text.c_str());
	fflush(stdout);
}
