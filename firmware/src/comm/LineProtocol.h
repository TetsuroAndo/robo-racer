#pragma once
#include <Stream.h>
#include "comm/Command.h"

namespace mc::comm {

class LineProtocol {
 public:
	bool poll(Stream &stream, RcCommand &out);

 private:
	static constexpr size_t kBufSize = 64;
	char _buf[kBufSize]{};
	size_t _len = 0;

	bool parseLine(const char *line, RcCommand &out);
};

} // namespace mc::comm
