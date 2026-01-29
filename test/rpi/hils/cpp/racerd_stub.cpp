#include <mc/proto/Proto.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <string>

static bool send_frame(int fd, mc::proto::Type type, uint16_t seq,
					   const uint8_t *payload, uint16_t payload_len) {
	uint8_t enc[mc::proto::MAX_FRAME_ENCODED];
	size_t enc_len = 0;
	if (!mc::proto::PacketWriter::build(enc, sizeof(enc), enc_len, type, 0, seq,
										payload, payload_len)) {
		return false;
	}
	return ::send(fd, enc, enc_len, 0) == (ssize_t)enc_len;
}

int main(int argc, char **argv) {
	if (argc < 2) {
		std::fprintf(stderr, "usage: %s <sock_path>\n", argv[0]);
		return 1;
	}
	const std::string sock_path = argv[1];

	int fd = ::socket(AF_UNIX, SOCK_SEQPACKET, 0);
	if (fd < 0) {
		perror("socket");
		return 1;
	}

	sockaddr_un addr{};
	addr.sun_family = AF_UNIX;
	std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s",
				  sock_path.c_str());
	if (::connect(fd, reinterpret_cast< sockaddr * >(&addr), sizeof(addr)) !=
		0) {
		perror("connect");
		::close(fd);
		return 1;
	}

	// MODE_SET AUTO
	uint8_t mode = 1;
	if (!send_frame(fd, mc::proto::Type::MODE_SET, 1, &mode, 1)) {
		::close(fd);
		return 1;
	}

	// DRIVE (steer=100, speed=200, ttl=50, dist=0)
	mc::proto::DrivePayload drive{};
	drive.steer_cdeg = 100;
	drive.speed_mm_s = 200;
	drive.ttl_ms_le = mc::proto::host_to_le16(50);
	drive.dist_mm_le = 0;
	if (!send_frame(fd, mc::proto::Type::DRIVE, 2,
					reinterpret_cast< const uint8_t * >(&drive),
					sizeof(drive))) {
		::close(fd);
		return 1;
	}

	::close(fd);
	return 0;
}
