#pragma once

namespace seriald_cfg {

static constexpr const char *DEFAULT_DEV = "/dev/ttyAMA0";
static constexpr int DEFAULT_BAUD = 921600;
static constexpr const char *DEFAULT_CONTROL_SOCK =
	"/tmp/roboracer/seriald.sock";
static constexpr const char *DEFAULT_TELEMETRY_SOCK =
	"/tmp/roboracer/seriald.telemetry.sock";
static constexpr int DEFAULT_TELEMETRY_TCP_PORT = 5001;
static constexpr const char *DEFAULT_COMPAT_CONTROL_SOCK =
	"/tmp/roboracer/seriald.sock";
static constexpr const char *DEFAULT_COMPAT_TELEMETRY_SOCK =
	"/tmp/roboracer/seriald.telemetry.sock";
static constexpr const char* DEFAULT_LOG = "./logs/seriald.log";

static constexpr int POLL_TIMEOUT_MS = 10;
static constexpr int TX_IDLE_US = 1000;
static constexpr int MAX_CLIENT_FDS = 64;

} // namespace seriald_cfg
