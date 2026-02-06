#pragma once

#include <csignal>
#include <cstdlib>
#include <iostream>

namespace mc::core {

namespace {
static volatile sig_atomic_t *s_stop_flag = nullptr;
static void (*s_extra_cb)(int) = nullptr;

static void signal_handler(int sig) {
	if (s_stop_flag)
		*s_stop_flag = 1;
	if (s_extra_cb)
		s_extra_cb(sig);
}
} // namespace

/** Set SIGINT/SIGTERM to set *stop_flag to 1. Optional extra_cb(sig) called
 * after. */
inline void setup_signal_handlers(volatile sig_atomic_t *stop_flag,
								  void (*extra_cb)(int) = nullptr) {
	s_stop_flag = stop_flag;
	s_extra_cb = extra_cb;
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
}

/** Restore cursor visibility (e.g. after ncurses-style UI). */
inline void show_cursor() { std::cout << "\x1b[?25h" << std::flush; }

/** Register show_cursor to run at exit. */
inline void register_atexit_show_cursor() { std::atexit(show_cursor); }

} // namespace mc::core
