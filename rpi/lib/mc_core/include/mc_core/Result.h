#pragma once
#include <cstdint>

namespace mc {

enum class Errc : uint8_t {
	Ok = 0,
	Timeout,
	Bus,
	Invalid,
	NotReady,
	Range,
	Internal
};

struct Result {
	Errc code;
	const char* msg;

	constexpr bool ok() const { return code == Errc::Ok; }

	static constexpr Result Ok() { return {Errc::Ok, ""}; }
	static constexpr Result Fail(Errc c, const char* m) { return {c, m}; }
};

} // namespace mc
