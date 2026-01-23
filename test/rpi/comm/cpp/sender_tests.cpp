#include "Sender.h"

#include <mc/proto/Proto.hpp>

#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

static const char *kSockPath = "/tmp/seriald_sender.sock";

int make_server_socket() {
	int fd = ::socket(AF_UNIX, MC_IPC_DEFAULT_SOCK_TYPE, 0);
	assert(fd >= 0);
	::unlink(kSockPath);
	sockaddr_un addr{};
	addr.sun_family = AF_UNIX;
	std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", kSockPath);
	const int rc = ::bind(fd, (sockaddr *)&addr, sizeof(addr));
	assert(rc == 0);
	const int lrc = ::listen(fd, 1);
	assert(lrc == 0);
	return fd;
}

bool recv_frame(int fd,
				std::array< uint8_t, mc::proto::MAX_FRAME_ENCODED > &buf,
				size_t &len, int timeout_ms) {
	fd_set rfds;
	FD_ZERO(&rfds);
	FD_SET(fd, &rfds);
	timeval tv{};
	tv.tv_sec = timeout_ms / 1000;
	tv.tv_usec = (timeout_ms % 1000) * 1000;
	const int r = ::select(fd + 1, &rfds, nullptr, nullptr, &tv);
	if (r <= 0)
		return false;
	len = (size_t)::recv(fd, buf.data(), buf.size(), 0);
	return len > 0;
}

bool decode(const std::array< uint8_t, mc::proto::MAX_FRAME_ENCODED > &buf,
			size_t len, mc::proto::Frame &out) {
	std::array< uint8_t, mc::proto::MAX_FRAME_DECODED > decoded{};
	return mc::proto::decode_one(buf.data(), len, out, decoded);
}

bool send_ack(int fd, uint16_t seq) {
	uint8_t enc[mc::proto::MAX_FRAME_ENCODED];
	size_t enc_len = 0;
	const bool ok = mc::proto::PacketWriter::build(
		enc, sizeof(enc), enc_len, mc::proto::Type::ACK, 0, seq, nullptr, 0);
	if (!ok)
		return false;
	const ssize_t sent = ::send(fd, enc, enc_len, 0);
	return sent > 0;
}

std::string hex_bytes(const uint8_t *p, size_t n) {
	static const char *hex = "0123456789abcdef";
	std::string s;
	for (size_t i = 0; i < n; ++i) {
		unsigned v = p[i];
		s.push_back(hex[v >> 4]);
		s.push_back(hex[v & 0x0F]);
	}
	return s;
}

} // namespace

/**
 * @brief
 *   Verify that Sender sends MODE_SET with ACK_REQ on startup and
 *   that the server can ACK it, clearing pending state.
 */
static void test_sender_mode_set_ack() {
	std::cout << "[TEST] sender_mode_set_ack\n";
	const int srv_fd = make_server_socket();
	Sender sender(kSockPath);
	const int cfd = ::accept(srv_fd, nullptr, nullptr);
	assert(cfd >= 0);

	std::array< uint8_t, mc::proto::MAX_FRAME_ENCODED > buf{};
	size_t len = 0;
	const bool got = recv_frame(cfd, buf, len, 500);
	assert(got);

	mc::proto::Frame f{};
	const bool ok = decode(buf, len, f);
	assert(ok);
	std::cout << "\tRX type=0x" << std::hex << (unsigned)f.type() << " flags=0x"
			  << (unsigned)f.flags() << " seq=0x" << f.seq() << std::dec
			  << " payload=" << hex_bytes(f.payload, f.payload_len) << "\n";

	assert(f.type() == (uint8_t)mc::proto::Type::MODE_SET);
	assert(f.flags() == mc::proto::FLAG_ACK_REQ);
	assert(f.payload_len == 1);

	// Send ACK back to Sender and let poll consume it.
	const uint16_t seq = f.seq();
	assert(send_ack(cfd, seq));

	sender.poll();
	::close(cfd);
	::close(srv_fd);
	std::cout << "\tACK sent for seq=0x" << std::hex << seq << std::dec << "\n";
}

/**
 * @brief
 *   Verify that DRIVE is not sent when auto mode is disabled.
 */
static void test_sender_auto_disabled_no_drive() {
	std::cout << "[TEST] sender_auto_disabled_no_drive\n";
	const int srv_fd = make_server_socket();
	Sender sender(kSockPath);
	const int cfd = ::accept(srv_fd, nullptr, nullptr);
	assert(cfd >= 0);

	// Drain initial MODE_SET
	std::array< uint8_t, mc::proto::MAX_FRAME_ENCODED > buf{};
	size_t len = 0;
	if (recv_frame(cfd, buf, len, 500)) {
		mc::proto::Frame f{};
		if (decode(buf, len, f) &&
			f.type() == (uint8_t)mc::proto::Type::MODE_SET &&
			(f.flags() & mc::proto::FLAG_ACK_REQ)) {
			send_ack(cfd, f.seq());
			sender.poll();
		}
	}

	sender.sendAutoMode(false);
	if (recv_frame(cfd, buf, len, 500)) {
		mc::proto::Frame f{};
		if (decode(buf, len, f) &&
			f.type() == (uint8_t)mc::proto::Type::MODE_SET &&
			(f.flags() & mc::proto::FLAG_ACK_REQ)) {
			send_ack(cfd, f.seq());
			sender.poll();
		}
	}

	// Now auto is disabled; send() should not emit DRIVE.
	sender.send(100, 10);
	bool saw_drive = false;
	const auto deadline =
		std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
	while (std::chrono::steady_clock::now() < deadline) {
		const bool got = recv_frame(cfd, buf, len, 50);
		if (!got)
			continue;
		mc::proto::Frame f{};
		if (!decode(buf, len, f))
			continue;
		if (f.type() == (uint8_t)mc::proto::Type::DRIVE) {
			saw_drive = true;
			break;
		}
		if (f.type() == (uint8_t)mc::proto::Type::MODE_SET &&
			(f.flags() & mc::proto::FLAG_ACK_REQ)) {
			send_ack(cfd, f.seq());
			sender.poll();
		}
	}
	std::cout << "\tEXPECT no DRIVE, actual=" << (saw_drive ? "drive" : "none")
			  << "\n";
	assert(!saw_drive);
	::close(cfd);
	::close(srv_fd);
}

/**
 * @brief
 *   Verify that PING is sent roughly at HEARTBEAT_INTERVAL_MS.
 */
static void test_sender_ping_interval() {
	std::cout << "[TEST] sender_ping_interval\n";
	const int srv_fd = make_server_socket();
	Sender sender(kSockPath);
	const int cfd = ::accept(srv_fd, nullptr, nullptr);
	assert(cfd >= 0);

	std::array< uint8_t, mc::proto::MAX_FRAME_ENCODED > buf{};
	size_t len = 0;

	// Drain initial MODE_SET
	recv_frame(cfd, buf, len, 500);

	sender.send(0, 0);
	std::this_thread::sleep_for(
		std::chrono::milliseconds(cfg::HEARTBEAT_INTERVAL_MS + 10));
	sender.send(0, 0);

	bool saw_ping = false;
	for (int i = 0; i < 3; ++i) {
		const bool got = recv_frame(cfd, buf, len, 200);
		if (!got)
			break;
		mc::proto::Frame f{};
		if (!decode(buf, len, f))
			continue;
		if (f.type() == (uint8_t)mc::proto::Type::PING)
			saw_ping = true;
		std::cout << "\tRX type=0x" << std::hex << (unsigned)f.type()
				  << std::dec << "\n";
	}
	std::cout << "\tEXPECT ping=true, actual=" << (saw_ping ? "true" : "false")
			  << "\n";
	assert(saw_ping);
	::close(cfd);
	::close(srv_fd);
}

/**
 * @brief
 *   Verify ACK retry occurs when ACK is not received.
 */
static void test_sender_ack_retry() {
	std::cout << "[TEST] sender_ack_retry\n";
	const int srv_fd = make_server_socket();
	Sender sender(kSockPath);
	const int cfd = ::accept(srv_fd, nullptr, nullptr);
	assert(cfd >= 0);

	std::array< uint8_t, mc::proto::MAX_FRAME_ENCODED > buf{};
	size_t len = 0;
	recv_frame(cfd, buf, len, 500); // initial MODE_SET

	sender.sendKill();
	int count = 0;
	const auto deadline =
		std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
	while (std::chrono::steady_clock::now() < deadline) {
		sender.poll();
		if (recv_frame(cfd, buf, len, 50)) {
			mc::proto::Frame f{};
			if (decode(buf, len, f) &&
				f.type() == (uint8_t)mc::proto::Type::KILL) {
				count++;
			}
		}
		std::this_thread::sleep_for(
			std::chrono::milliseconds(cfg::ACK_TIMEOUT_MS + 5));
	}
	std::cout << "\tEXPECT retries>=2, actual=" << count << "\n";
	assert(count >= 2);
	::close(cfd);
	::close(srv_fd);
}

int main() {
	test_sender_mode_set_ack();
	test_sender_auto_disabled_no_drive();
	test_sender_ping_interval();
	test_sender_ack_retry();
	std::cout << "sender tests ok\n";
	return 0;
}
