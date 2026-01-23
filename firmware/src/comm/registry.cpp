#include "registry.h"
#include <string.h>

namespace mc {

Registry &Registry::instance() {
	static Registry inst;
	return inst;
}

Registry::Registry() : _n(0) { memset(_e, 0, sizeof(_e)); }

bool Registry::add(uint8_t type, IHandler *h) {
	for (int i = 0; i < _n; i++) {
		if (_e[i].type == type) {
			_e[i].h = h;
			return true;
		}
	}
	if (_n >= MAX)
		return false;
	_e[_n++] = {type, h};
	return true;
}

IHandler *Registry::get(uint8_t type) {
	for (int i = 0; i < _n; i++)
		if (_e[i].type == type)
			return _e[i].h;
	return nullptr;
}

} // namespace mc
