#include "comm/LineProtocol.h"
#include "log/Log.h"
#include <common/Math.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>

namespace mc::comm {

static float normalizeInput(float v) {
	if (fabsf(v) <= 1.2f)
		return v;
	return v / 1000.0f;
}

bool LineProtocol::parseLine(const char *line, RcCommand &out) {
	float values[2] = {0.0f, 0.0f};
	int found = 0;
	const char *p = line;

	while (*p && found < 2) {
		while (*p && !isdigit((unsigned char)*p) && *p != '+' && *p != '-' &&
			   *p != '.') {
			++p;
		}
		if (!*p)
			break;
		char *end = nullptr;
		const float v = strtof(p, &end);
		if (end == p)
			break;
		values[found++] = v;
		p = end;
	}

	if (found < 2)
		return false;

	out.throttle = mc::clamp(normalizeInput(values[0]), -1.0f, 1.0f);
	out.steer = mc::clamp(normalizeInput(values[1]), -1.0f, 1.0f);
	return true;
}

bool LineProtocol::poll(Stream &stream, RcCommand &out) {
	bool updated = false;
	while (stream.available() > 0) {
		const int c = stream.read();
		if (c < 0)
			break;

		if (c == '\n' || c == '\r') {
			if (_len > 0) {
				_buf[_len] = '\0';
				if (parseLine(_buf, out))
					updated = true;
				else
					MC_LOGT(mc::log::Topic::Comm, "drop line: %s", _buf);
				_len = 0;
			}
			continue;
		}

		if (_len + 1 >= kBufSize) {
			_len = 0;
			MC_LOGW(mc::log::Topic::Comm, "line overflow, drop buffer");
			continue;
		}

		_buf[_len++] = (char)c;
	}

	return updated;
}

} // namespace mc::comm
