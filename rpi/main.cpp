#include "LidarReceiver.h"
#include "Process.h"
#include "Sender.h"
#include "lidar_to_esp.h"
#include <csignal>
#include <cstdlib>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }

int main(int argc, char **argv) {
	signal(SIGINT, on_sig);
	signal(SIGTERM, on_sig);

	const char *lidar_dev = (argc >= 2) ? argv[1] : "/dev/ttyAMA2";
	int lidar_baud = (argc >= 3) ? std::atoi(argv[2]) : 460800;
	const char *esp_dev = (argc >= 4) ? argv[3] : "/dev/ttyAMA0";

	LidarReceiver lidarReceiver(lidar_dev, lidar_baud);
	Process process;
	Sender sender(esp_dev);

	while (!g_stop) {
		const std::vector< LidarData > &res = lidarReceiver.receive();
		const ProcResult procResult = process.proc(res);
		usleep(200 * 1000);
		sender.send(procResult.speed, procResult.angle);
	}
	return 0;
}
